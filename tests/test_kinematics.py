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
