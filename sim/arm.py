import math

L1 = 170
L2 = 170

# Angle convention (all zero = arm pointing straight up, base facing +y):
#   q1  base yaw about z, positive turns from +y toward -x (counterclockwise seen from above)
#   q2  shoulder tilt from vertical, positive leans forward
#   q3  elbow bend relative to link 1, positive bends further forward
# In the arm's plane, r = horizontal distance forward, z = height.

def fk(q2_deg, q3_deg):
    q2 = math.radians(q2_deg)
    q3 = math.radians(q3_deg)
    r = L1 * math.sin(q2) + L2 * math.sin(q2 + q3)
    z = L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
    return r, z

def ik(r, z, elbow_up=True):
    d2 = math.sqrt(r**2 + z**2)
    c3 = (d2**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c3) > 1 + 1e-9:
        return None                      # target out of reach
    c3 = max(-1.0, min(1.0, c3))         # clean up tiny rounding errors
    q3 = math.acos(c3)
    if not elbow_up:
        q3 = -q3
    beta = math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    q2 = math.atan2(r, z) - beta         # angle measured from vertical
    return math.degrees(q2), math.degrees(q3)

def fk3(q1_deg, q2_deg, q3_deg):
    q1 = math.radians(q1_deg)
    r, z = fk(q2_deg, q3_deg)
    x = -r * math.sin(q1)
    y = r * math.cos(q1)
    return x, y, z

Q_MIN = -100   # joint limits (deg), same for all three joints
Q_MAX = 100

def wrap180(a):
    return (a + 180) % 360 - 180         # keep angle in [-180, 180)

def within_limits(q1, q2, q3):
    return all(Q_MIN <= q <= Q_MAX for q in (q1, q2, q3))

def ik3(x, y, z, elbow_up=True):
    q1 = math.degrees(math.atan2(-x, y))
    r = math.hypot(x, y)

    # up to 4 solutions: elbow up/down, and base facing the target or
    # turned 180 deg with the arm reaching back over the top
    candidates = []
    for elbow in (elbow_up, not elbow_up):           # preferred elbow first
        for base, reach in ((q1, r), (wrap180(q1 + 180), -r)):
            sol = ik(reach, z, elbow)
            if sol is not None:
                q2, q3 = sol
                candidates.append((base, wrap180(q2), q3))

    if not candidates:
        raise ValueError(f"target ({x}, {y}, {z}) is out of reach")
    for q in candidates:
        if within_limits(*q):
            return q
    raise ValueError(f"target ({x}, {y}, {z}) needs angles outside {Q_MIN}..{Q_MAX} deg")

if __name__ == "__main__":               # only run tests when run directly, not on import
    import random

    assert all(abs(a - b) < 1e-9 for a, b in zip(fk3(0, 0, 0), (0, 0, L1 + L2)))   # zero = straight up

    worst = 0.0
    for i in range(1000):
        q1 = random.uniform(Q_MIN, Q_MAX)
        q2 = random.uniform(Q_MIN, Q_MAX)
        q3 = random.uniform(Q_MIN, Q_MAX)

        x, y, z = fk3(q1, q2, q3)
        sol3 = ik3(x, y, z, elbow_up=(q3 > 0))   # every pose here is reachable

        assert within_limits(*sol3), sol3
        x2, y2, z2 = fk3(*sol3)
        err3 = math.hypot(x2 - x, y2 - y, z2 - z)
        worst = max(worst, err3)

    print("worst error (mm):", worst)

    for target in [(500, 0, 0), (0, -200, 0)]:   # too far / behind the base
        try:
            ik3(*target)
        except ValueError as e:
            print("error:", e)
