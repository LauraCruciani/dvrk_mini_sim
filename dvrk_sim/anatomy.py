"""Realistic anatomy: a laparoscopic scene reconstructed from a real video frame.

The data come from the SurgAware exports (folder surgaware_ply_export/, not
part of this repository: Cholec80-derived textures, CC BY-NC-SA 4.0, for
non-commercial teaching only). Each scene is a 2.5D surface seen from the
left camera of the original laparoscope: world frame = that camera
(x right, y down, z viewing direction), metres, one organ label per vertex.

    python -m dvrk_sim.anatomy            build the assets (once, a few seconds)

writes web/assets/anatomy/ (mesh + photo texture for the endoscope view) and
grid.npz (depth, normal and organ maps used by the robots for collisions).

The scene is placed so that the ECM at its home pose sees exactly the
original photo: the mesh frame is the camera frame of the ECM at ECM_HOME.
Because the surface is seen from one viewpoint only, the camera arm is kept
in poses from which the whole view still falls on the reconstructed surface
(see Anatomy.view_ok): you never see its borders.
"""
import json
import os
import struct
import sys

import numpy as np

from .kinematics import ecm_forward, ECM_HOME

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "surgaware_ply_export")
OUT = os.path.join(ROOT, "web", "assets", "anatomy")
SCENE = "v03_hero"
PHOTO_U, PHOTO_V = 0.394, 0.221          # half extent of the photo in normalised image coordinates (x/z, y/z)
GRID = (720, 480)                        # collision maps (u, v)
TEX_PX = 1100
VIEW_MARGIN = (0.12, 0.10)               # how far past the photo the camera may look                            # texture pixels per unit of u (the photo has ~1084)


# --------------------------------------------------------------------- PLY
def read_ply(path):
    """Binary little-endian PLY with x y z red green blue label (+ triangle faces)."""
    with open(path, "rb") as f:
        head = b""
        while not head.endswith(b"end_header\n"):
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: no PLY header")
            head += line
        h = head.decode("ascii", "replace")
        nv = int(h.split("element vertex ")[1].split()[0])
        nf = int(h.split("element face ")[1].split()[0]) if "element face" in h else 0
        vt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                       ("r", "u1"), ("g", "u1"), ("b", "u1"), ("l", "u1")])
        v = np.frombuffer(f.read(nv * vt.itemsize), vt)
        faces = None
        if nf:
            ft = np.dtype([("n", "u1"), ("i", "<i4", (3,))])
            faces = np.frombuffer(f.read(nf * ft.itemsize), ft)
            if not (faces["n"] == 3).all():
                raise ValueError(f"{path}: only triangles are supported")
            faces = faces["i"].astype(np.int64)
    names = {}
    for line in h.splitlines():
        if line.startswith("comment classes"):
            for item in line.split("classes", 1)[1].split(","):
                k, _, name = item.strip().partition("=")
                names[int(k)] = name
    P = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    C = np.stack([v["r"], v["g"], v["b"]], 1)
    return P, C, v["l"].copy(), faces, names


def anchor():
    """World pose of the mesh frame: the camera of the ECM at its home pose."""
    return ecm_forward(ECM_HOME)


def inpaint_dark(C, F, threshold=60, iterations=60):
    """Replace near-black vertex colours (holes of the padding) with the average of their neighbours."""
    C = C.astype(float)
    bad = C.sum(1) < threshold
    edges = np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    edges = np.vstack([edges, edges[:, ::-1]])
    for _ in range(iterations):
        if not bad.any():
            break
        good = ~bad[edges[:, 1]]
        acc = np.zeros_like(C); cnt = np.zeros(len(C))
        np.add.at(acc, edges[good, 0], C[edges[good, 1]])
        np.add.at(cnt, edges[good, 0], 1)
        fix = bad & (cnt > 0)
        C[fix] = acc[fix] / cnt[fix][:, None]
        bad &= ~fix
    return C


# --------------------------------------------------------------- raster
def rasterize(u, v, z, attrs, faces, gu, gv, chunk=20000):
    """Z-buffer the triangles (u, v = normalised image coordinates, z = depth) on the grid gu x gv.

    attrs: (N, k) per-vertex values, interpolated with barycentric weights.
    Returns depth (H, W) with nan where nothing is drawn, and the attributes (H, W, k)
    of the surface nearest to the camera at each pixel.
    """
    W, H = len(gu), len(gv)
    px = (u - gu[0]) / (gu[-1] - gu[0]) * (W - 1)
    py = (v - gv[0]) / (gv[-1] - gv[0]) * (H - 1)
    xs, ys = px[faces], py[faces]
    x0, x1 = np.ceil(xs.min(1)).astype(int), np.floor(xs.max(1)).astype(int)
    y0, y1 = np.ceil(ys.min(1)).astype(int), np.floor(ys.max(1)).astype(int)
    S = int(min(16, max(1, (x1 - x0).max() + 1, (y1 - y0).max() + 1)))
    off = np.arange(S)
    pix, tri, bary, dep = [], [], [], []
    for s in range(0, len(faces), chunk):
        sl = slice(s, s + chunk)
        X = x0[sl, None, None] + off[None, None, :]
        Y = y0[sl, None, None] + off[None, :, None]
        X, Y = np.broadcast_arrays(X, Y)
        ax, ay = xs[sl, 0, None, None], ys[sl, 0, None, None]
        bx, by = xs[sl, 1, None, None], ys[sl, 1, None, None]
        cx, cy = xs[sl, 2, None, None], ys[sl, 2, None, None]
        den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        den = np.where(np.abs(den) < 1e-12, np.nan, den)
        l0 = ((by - cy) * (X - cx) + (cx - bx) * (Y - cy)) / den
        l1 = ((cy - ay) * (X - cx) + (ax - cx) * (Y - cy)) / den
        l2 = 1 - l0 - l1
        m = (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9) & (X >= 0) & (X < W) & (Y >= 0) & (Y < H)
        m &= (X <= x1[sl, None, None]) & (Y <= y1[sl, None, None])
        t, j, i = np.nonzero(m)
        T = t + s
        L = np.stack([l0[t, j, i], l1[t, j, i], l2[t, j, i]], 1)
        pix.append(Y[t, j, i] * W + X[t, j, i]); tri.append(T); bary.append(L)
        dep.append(np.einsum("ij,ij->i", L, z[faces[T]]))
    pix, tri, bary, dep = (np.concatenate(a) for a in (pix, tri, bary, dep))
    order = np.lexsort((dep, pix))                              # nearest surface first for each pixel
    first = order[np.r_[True, pix[order][1:] != pix[order][:-1]]]
    depth = np.full(W * H, np.nan)
    out = np.zeros((W * H, attrs.shape[1]))
    depth[pix[first]] = dep[first]
    out[pix[first]] = np.einsum("ij,ijk->ik", bary[first], attrs[faces[tri[first]]])
    return depth.reshape(H, W), out.reshape(H, W, -1)


# ------------------------------------------------------------------- build
def build(scene=SCENE, data=DATA, out=OUT, log=print):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mimg

    src = os.path.join(data, scene)
    P, C, L, F, names = read_ply(os.path.join(src, "mesh_t000.ply"))
    Q, QC, _, _, _ = read_ply(os.path.join(src, "pointcloud_left_t000.ply"))
    C = inpaint_dark(C, F)
    log(f"[anatomy] {scene}: {len(P)} vertices, {len(F)} triangles, organs {names}")
    u, v = P[:, 0] / P[:, 2], P[:, 1] / P[:, 2]
    u0, u1, v0, v1 = u.min(), u.max(), v.min(), v.max()

    # normals of the 3D surface, facing the camera
    n = np.zeros_like(P)
    fn = np.cross(P[F[:, 1]] - P[F[:, 0]], P[F[:, 2]] - P[F[:, 0]])
    for k in range(3):
        np.add.at(n, F[:, k], fn)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    n[np.einsum("ij,ij->i", n, P) > 0] *= -1

    # collision maps on a regular (u, v) grid
    gu = np.linspace(u0, u1, GRID[0])
    gv = np.linspace(v0, v1, GRID[1])
    onehot = np.zeros((len(P), 16))
    onehot[np.arange(len(P)), L] = 1.0
    depth, att = rasterize(u, v, P[:, 2], np.hstack([n, onehot]), F, gu, gv)
    normal = att[..., :3]
    normal /= np.maximum(np.linalg.norm(normal, axis=-1, keepdims=True), 1e-12)
    label = np.where(np.isfinite(depth), att[..., 3:].argmax(-1), 0).astype(np.uint8)
    os.makedirs(out, exist_ok=True)
    np.savez_compressed(os.path.join(out, "grid.npz"), u=gu, v=gv, depth=depth.astype(np.float32), normal=normal.astype(np.float32),
                        label=label, names=json.dumps(names))

    # texture over the whole mesh: the photo inside its frame, the mesh colours around it
    W = int((u1 - u0) * TEX_PX)
    H = int((v1 - v0) * TEX_PX)
    tu = np.linspace(u0, u1, W)
    tv = np.linspace(v0, v1, H)
    TU, TV = np.meshgrid(tu, tv)
    _, tex = rasterize(u, v, P[:, 2], C.astype(float), F, tu, tv)
    qu, qv = Q[:, 0] / Q[:, 2], Q[:, 1] / Q[:, 2]           # the photo: one point per pixel
    step = lambda a: np.median(np.diff(np.unique(np.round(a, 7)))[:2000])
    su = np.median(np.diff(np.sort(qu))[np.diff(np.sort(qu)) > 1e-6])
    sv = np.median(np.diff(np.sort(qv))[np.diff(np.sort(qv)) > 1e-6])
    pw = int(round((qu.max() - qu.min()) / su)) + 1
    ph = int(round((qv.max() - qv.min()) / sv)) + 1
    col = np.clip(np.rint((qu - qu.min()) / su), 0, pw - 1).astype(int)
    row = np.clip(np.rint((qv - qv.min()) / sv), 0, ph - 1).astype(int)
    photo = np.zeros((ph, pw, 3))
    have = np.zeros((ph, pw), bool)
    photo[row, col] = QC
    have[row, col] = True
    for _ in range(4):                                       # fill isolated missing pixels from neighbours
        hole = ~have
        if not hole.any():
            break
        acc = np.zeros_like(photo); cnt = np.zeros(have.shape)
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            sh = np.roll(np.roll(have, dy, 0), dx, 1)
            acc += np.roll(np.roll(photo, dy, 0), dx, 1) * sh[..., None]; cnt += sh
        fill = hole & (cnt > 0)
        photo[fill] = acc[fill] / cnt[fill][:, None]
        have |= fill
    log(f"[anatomy] photo {pw}x{ph}, {(~have).sum()} pixels missing")
    pc = np.rint((TU - qu.min()) / su).astype(int)
    pr = np.rint((TV - qv.min()) / sv).astype(int)
    inside = (pc >= 0) & (pc < pw) & (pr >= 0) & (pr < ph)
    inside[inside] &= have[pr[inside], pc[inside]]
    # the mesh colours are de-lit and darker than the photo: match their brightness
    gain = photo[have].mean(0) / np.maximum(tex[inside].mean(0), 1.0)
    tex = np.clip(tex * gain, 0, 255)
    # feather the edge of the photo over ~3 % of its width
    du = np.minimum(TU - qu.min(), qu.max() - TU) / (qu.max() - qu.min())
    dv = np.minimum(TV - qv.min(), qv.max() - TV) / (qu.max() - qu.min())
    w = np.clip(np.minimum(du, dv) / 0.03, 0, 1) * inside
    ph_s = np.zeros_like(tex)
    ph_s[inside] = photo[pr[inside], pc[inside]]
    tex = tex * (1 - w[..., None]) + ph_s * w[..., None]
    mimg.imsave(os.path.join(out, "texture.png"), (tex / 255.0).astype(np.float32))

    # mesh for the browser: positions (mesh frame), uv, triangles, organ labels
    uv = np.stack([(u - u0) / (u1 - u0), 1.0 - (v - v0) / (v1 - v0)], 1)
    with open(os.path.join(out, "mesh.bin"), "wb") as f:
        f.write(P.astype("<f4").tobytes())
        f.write(uv.astype("<f4").tobytes())
        f.write(F.astype("<u4").tobytes())
        f.write(L.astype("u1").tobytes())
    c, Rc = anchor()
    meta = {"scene": scene, "vertices": len(P), "triangles": len(F), "organs": names,
            "anchor_p": c.tolist(), "anchor_R": Rc.tolist(), "photo": [PHOTO_U, PHOTO_V],
            "license": "Cholec80 / CholecSeg8k derived, CC BY-NC-SA 4.0, non-commercial use only"}
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    log(f"[anatomy] wrote {out} (texture {W}x{H})")


def available():
    return os.path.exists(os.path.join(OUT, "grid.npz")) and os.path.exists(os.path.join(OUT, "meta.json"))


def ensure(log=print):
    """Build the assets if the data are there and they are missing. True if the scene can be used."""
    if available():
        return True
    if os.path.isdir(os.path.join(DATA, SCENE)):
        build(log=log)
        return True
    return False


# --------------------------------------------------------------- queries
class Anatomy:
    """Collision queries against the reconstructed surface, in the world frame."""

    def __init__(self, out=OUT):
        g = np.load(os.path.join(out, "grid.npz"))
        self.gu, self.gv = g["u"], g["v"]
        self.depth, self.normal, self.label = g["depth"], g["normal"].astype(np.float64), g["label"]
        self.names = {int(k): n for k, n in json.loads(str(g["names"])).items()}
        self.c, self.R = anchor()
        self.du = (self.gu[-1] - self.gu[0]) / (len(self.gu) - 1)
        self.dv = (self.gv[-1] - self.gv[0]) / (len(self.gv) - 1)

    def _lookup(self, u, v):
        """Bilinear depth, normal (mesh frame) and nearest label at normalised coordinates; nan outside."""
        x = (u - self.gu[0]) / self.du
        y = (v - self.gv[0]) / self.dv
        ok = (x >= 0) & (x <= len(self.gu) - 1) & (y >= 0) & (y <= len(self.gv) - 1)
        x = np.clip(x, 0, len(self.gu) - 1.001)
        y = np.clip(y, 0, len(self.gv) - 1.001)
        i, j = x.astype(int), y.astype(int)
        a, b = (x - i)[..., None], (y - j)[..., None]
        D = self.depth[..., None]
        d = ((1 - a) * (1 - b) * D[j, i] + a * (1 - b) * D[j, i + 1]
             + (1 - a) * b * D[j + 1, i] + a * b * D[j + 1, i + 1])[..., 0]
        N = self.normal
        n = (1 - a) * (1 - b) * N[j, i] + a * (1 - b) * N[j, i + 1] + (1 - a) * b * N[j + 1, i] + a * b * N[j + 1, i + 1]
        lab = self.label[np.rint(y).astype(int), np.rint(x).astype(int)]
        d = np.where(ok, d, np.nan)
        return d, n, lab

    def surface(self, pts, radii):
        """For world points: gap to the surface along its normal (m, nan if not above it), world normal, organ."""
        pts = np.atleast_2d(pts)
        q = (pts - self.c) @ self.R                        # world -> mesh (camera) frame
        z = np.maximum(q[:, 2], 1e-6)
        d, n, lab = self._lookup(q[:, 0] / z, q[:, 1] / z)
        ray = q / np.linalg.norm(q, axis=1, keepdims=True)
        cos = np.abs(np.einsum("ij,ij->i", n, ray))
        gap = (d - q[:, 2]) * np.clip(cos, 0.2, 1.0) - np.asarray(radii)
        gap = np.where(q[:, 2] > 0, gap, np.nan)
        return gap, n @ self.R.T, [self.names.get(int(k), "tissue") for k in lab]

    def project(self, p, margin):
        """Move a world point that is inside the surface back onto it (plus margin), along the normal."""
        gap, n, _ = self.surface(p, [margin])
        if np.isfinite(gap[0]) and gap[0] < 0:
            return np.asarray(p, dtype=float) - gap[0] * n[0]
        return np.asarray(p, dtype=float)

    def view_ok(self, c, Rc, half=(PHOTO_U, PHOTO_V), steps=200, reach=0.3):
        """True if the whole camera view falls on the reconstructed surface, near the original photo.

        Every ray of the view frustum must meet the surface coming from in front of it, at a point
        that the original camera saw within the photo plus VIEW_MARGIN: you may look a little past
        the photo (the blurred surroundings, like the vignetting of a laparoscope), never at the
        borders of the reconstruction.
        """
        cm = self.R.T @ (np.asarray(c, dtype=float) - self.c)
        Rm = self.R.T @ np.asarray(Rc, dtype=float)
        xs, ys = np.linspace(-half[0], half[0], 7), np.linspace(-half[1], half[1], 5)
        X, Y = np.meshgrid(xs, ys)
        dirs = np.stack([X.ravel(), Y.ravel(), np.ones(X.size)], 1) @ Rm.T
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        t = np.linspace(0.0, reach, steps)
        pts = cm + dirs[:, None, :] * t[None, :, None]               # rays x steps x 3
        z = np.maximum(pts[..., 2], 1e-4)
        u, v = pts[..., 0] / z, pts[..., 1] / z
        d, _, _ = self._lookup(u, v)
        d = np.where(pts[..., 2] > 1e-4, d, np.nan)
        hit = np.isfinite(d) & (pts[..., 2] >= d)
        if not hit.any(1).all():
            return False                                             # a ray never meets the surface
        k = hit.argmax(1)
        r = np.arange(len(k))
        if (k == 0).any() or not np.isfinite(d[r, k - 1]).all():
            return False                                             # it came from behind or from outside
        lu, lv = half[0] + VIEW_MARGIN[0], half[1] + VIEW_MARGIN[1]
        return bool((np.abs(u[r, k]) <= lu).all() and (np.abs(v[r, k]) <= lv).all())

    def start_tip(self, side, standoff=0.015):
        """A safe start point for an instrument: left over the gallbladder, right over the liver.

        Chosen so that the clevis and the jaws (even wide open) are clear of every organ.
        """
        uu, vv = (-0.08, 0.02) if side < 0 else (0.08, 0.02)
        d, _, _ = self._lookup(np.array([uu]), np.array([vv]))
        q = np.array([uu, vv, 1.0]) * (d[0] - standoff)
        return self.c + self.R @ q


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else SCENE)
