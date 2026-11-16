import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from dvrk_sim.kinematics import forward, inverse, LOWER, UPPER, mat_to_quat, quat_to_mat


def test_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(2000):
        q = LOWER + (UPPER - LOWER) * rng.random(6)
        q[4:] *= 0.95                     # stay away from the wrist singularity at +-pi/2
        p, R, _ = forward(q)
        q2, ok = inverse(p, R)
        assert ok, (q, q2)
        assert np.allclose(q, q2, atol=1e-7), (q, q2)


def test_rcm_on_shaft():
    rng = np.random.default_rng(1)
    for _ in range(200):
        q = LOWER + (UPPER - LOWER) * rng.random(6)
        _, _, wrist = forward(q)
        # the shaft is the segment from the RCM (origin) to the wrist: origin is on it by construction
        assert abs(np.linalg.norm(wrist) - q[2]) < 1e-12


def test_quaternions():
    rng = np.random.default_rng(2)
    for _ in range(500):
        q = LOWER + (UPPER - LOWER) * rng.random(6)
        _, R, _ = forward(q)
        assert np.allclose(quat_to_mat(mat_to_quat(R)), R, atol=1e-9)


if __name__ == "__main__":
    test_round_trip(); test_rcm_on_shaft(); test_quaternions()
    print("all kinematics tests passed")


def test_mtm():
    from dvrk_sim.kinematics import mtm_inverse, mtm_forward_points
    rng = np.random.default_rng(3)
    for _ in range(2000):
        p = rng.uniform([-0.15, -0.10, -0.12], [0.15, 0.10, 0.12])
        q, ok = mtm_inverse(p, [0, 0, 0])
        assert ok, p
        _, _, hand = mtm_forward_points(q)
        assert np.allclose(hand, p, atol=1e-9), (p, hand)


if __name__ == "__main__":
    test_mtm()
    print("MTM tests passed")


def test_segment_distances():
    from dvrk_sim.kinematics import segment_distances
    rng = np.random.default_rng(4)
    A1, B1, A2, B2 = (rng.normal(size=(5, 3)) for _ in range(4))
    d = segment_distances(A1, B1, A2, B2)
    s = np.linspace(0, 1, 401)
    for i in range(5):
        for j in range(5):
            P = A1[i] + (B1[i] - A1[i]) * s[:, None]
            Q = A2[j] + (B2[j] - A2[j]) * s[:, None]
            brute = np.min(np.linalg.norm(P[:, None] - Q[None], axis=-1))
            assert d[i, j] <= brute + 1e-12 and brute - d[i, j] < 5e-3, (d[i, j], brute)
    # parallel and touching segments
    assert abs(segment_distances([[0, 0, 0]], [[1, 0, 0]], [[0, 1, 0]], [[1, 1, 0]])[0, 0] - 1) < 1e-12
    assert segment_distances([[0, 0, 0]], [[1, 0, 0]], [[0.5, -1, 0]], [[0.5, 1, 0]])[0, 0] < 1e-12


def test_home_is_free():
    from dvrk_sim.kinematics import (PSM_BASES, PSM_HOME, ECM_HOME, ecm_forward, instrument_capsules,
                                     instrument_clearances, capsule_clearance, wall_height, ENDOSCOPE_R)
    caps = {}
    for a, base in PSM_BASES.items():
        tip, R, _ = forward(PSM_HOME[a])
        caps[a] = instrument_capsules(tip + base, R, 0.6, base)
        assert min(instrument_clearances(caps[a])) > 0, a
    assert capsule_clearance(caps["PSM1"], caps["PSM2"]) > 0
    p, _ = ecm_forward(ECM_HOME)
    assert wall_height(p[0], p[1]) - p[2] - ENDOSCOPE_R > 0


if __name__ == "__main__":
    test_segment_distances(); test_home_is_free()
    print("collision geometry tests passed")


def test_lnd_cad_matches_tip():
    """The CAD links of the Large Needle Driver end exactly on the simulator's tip pose."""
    from dvrk_sim.kinematics import JAW_LENGTH
    from dvrk_sim.lnd import link_poses, TIP, C
    rng = np.random.default_rng(5)
    for _ in range(300):
        q = np.array([*rng.uniform(-1, 1, 2), 0.1, *rng.uniform(-1.3, 1.3, 3)])
        tip, R, _ = forward(q)
        w, Ry = link_poses(q, 0.4, np.zeros(3))["yaw"]
        assert np.allclose(Ry @ TIP, R @ C, atol=1e-12)
        assert np.allclose(w + Ry @ [0, JAW_LENGTH, 0], tip, atol=1e-12)


def test_anatomy_if_built():
    from dvrk_sim import anatomy
    from dvrk_sim.kinematics import ecm_forward, ECM_HOME
    if not anatomy.available():
        print("anatomy not built: skipped")
        return
    a = anatomy.Anatomy()
    assert a.view_ok(*ecm_forward(ECM_HOME))                     # the original photo is a valid view
    assert not a.view_ok(*ecm_forward(ECM_HOME + [0.3, 0, 0, 0]))  # looking far away shows the borders
    for side in (-1, 1):
        gap, _, _ = a.surface(a.start_tip(side), [0.0])
        assert gap[0] > 0.005


if __name__ == "__main__":
    test_lnd_cad_matches_tip(); test_anatomy_if_built()
    print("CAD and anatomy tests passed")
