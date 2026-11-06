import math

from arm import ik3

HIP_HEIGHT = 164   # mm, shoulder joints above the ground (z = 0)
SPACING = 68       # mm between the two shoulders along y

# Legs stick out sideways: front arm to +x, back arm to -x. The back arm is a mirror
# image of the front arm (reflected across the yz plane), so the same angles give
# mirrored poses. Both face +y at zero. mirror flips x between the body frame and
# that arm's own frame.
LEGS = [
    {"name": "back",  "hip": (0, 0, HIP_HEIGHT),       "mirror": -1},
    {"name": "front", "hip": (0, SPACING, HIP_HEIGHT), "mirror": +1},
]

STRIDE = 100       # mm a foot travels along y per cycle
LIFT = 30          # mm foot clearance at the top of a swing
REACH_OUT = 215    # mm the foot sits out to the side of its hip (along x)
FOOT_Y = 40        # mm the middle of the stride sits ahead of the hip; keeps q1 inside
                   # the +/-100 deg limit, since straight out sideways is already q1 = -90
PERIOD = 1.5       # s per gait cycle

# duty = fraction of the cycle a foot is on the ground
# offset = where in the cycle each leg starts
GAITS = {
    "walk": {"duty": 0.6, "offset": {"back": 0.0, "front": 0.5}},   # alternate, one foot always down
    "push": {"duty": 0.5, "offset": {"back": 0.0, "front": 0.0}},   # both push together, then glide
}

def body_speed(gait):
    # during stance the foot slides back STRIDE mm in duty*PERIOD seconds;
    # moving the body forward at that speed keeps the planted foot still on the ground
    return STRIDE / (GAITS[gait]["duty"] * PERIOD)

def foot_offset(phase, duty):
    """Foot position along one cycle, relative to the middle of the stride (body frame).
    Returns (dy, dz, on_ground). phase in [0, 1)."""
    if phase < duty:                                  # stance: slide back in a straight line
        s = phase / duty
        return STRIDE * (0.5 - s), 0.0, True
    s = (phase - duty) / (1 - duty)                   # swing: lift and bring forward
    dy = STRIDE * (-0.5 + (1 - math.cos(math.pi * s)) / 2)   # eases in and out
    dz = LIFT * math.sin(math.pi * s)
    return dy, dz, False

def leg_targets(t, gait):
    """For every leg at time t: the foot target in that arm's own frame, and its stance/swing state."""
    g = GAITS[gait]
    out = []
    for leg in LEGS:
        m = leg["mirror"]
        phase = (t / PERIOD + g["offset"][leg["name"]]) % 1.0
        dy, dz, on_ground = foot_offset(phase, g["duty"])
        # body frame, relative to the hip: out to the side, stepping along y
        foot_body = (m * REACH_OUT, FOOT_Y + dy, -HIP_HEIGHT + dz)
        local = (m * foot_body[0], foot_body[1], foot_body[2])   # mirror into the arm's frame
        out.append((leg, local, on_ground))
    return out

def leg_angles(t, gait):
    """Joint angles (q1, q2, q3) for every leg at time t. Raises ValueError if a pose is unreachable."""
    return [ik3(*local) for _, local, _ in leg_targets(t, gait)]

if __name__ == "__main__":               # check the whole cycle is reachable for every gait
    for gait in GAITS:
        steps = 200
        for i in range(steps):
            leg_angles(PERIOD * i / steps, gait)
        print(f"{gait}: all {steps} poses reachable, body speed {body_speed(gait):.0f} mm/s")
