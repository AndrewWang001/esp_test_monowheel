import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import RadioButtons

from arm import L1, L2, ik3
from gait import LEGS, GAITS, SPACING, HIP_HEIGHT, leg_targets, body_speed

DT = 0.03                                # s per animation frame
VIEW = 300                               # mm shown either side of the body centre along y
COLORS = {"back": "tab:blue", "front": "tab:orange"}

def joint_positions(leg, body_y, q1_deg, q2_deg, q3_deg):
    # same math as fk3, but keeps the elbow point too so we can draw both links
    q1, q2, q3 = map(math.radians, (q1_deg, q2_deg, q3_deg))
    r_elbow = L1 * math.sin(q2)
    z_elbow = L1 * math.cos(q2)
    r_hand = r_elbow + L2 * math.sin(q2 + q3)
    z_hand = z_elbow + L2 * math.cos(q2 + q3)
    hx, hy = -math.sin(q1), math.cos(q1)  # unit vector the base is facing (arm frame)
    m = leg["mirror"]
    hip_x, hip_y, hip_z = leg["hip"]
    hip = (hip_x, body_y + hip_y, hip_z)
    # arm frame -> world: flip x for the mirrored arm, then shift to the hip
    elbow = (hip[0] + m * r_elbow * hx, hip[1] + r_elbow * hy, hip[2] + z_elbow)
    hand = (hip[0] + m * r_hand * hx, hip[1] + r_hand * hy, hip[2] + z_hand)
    return [hip, elbow, hand]

# --- 3D axes ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(projection="3d")
fig.subplots_adjust(bottom=0.15)

ax.set_xlim(-300, 300)
ax.set_zlim(0, 350)                      # z = 0 is the ground
ax.set_box_aspect((600, 2 * VIEW, 350))  # equal scale so links look the right length
ax.set_xlabel("x (mm)")
ax.set_ylabel("y (mm, forward)")
ax.set_zlabel("z (mm)")

# ground ticks every 50 mm so you can see the body moving past them
# (redrawn each frame, since 3D plots don't clip points outside the axis limits)
ground, = ax.plot([], [], [], ".", markersize=4, color="0.6")
body_line, = ax.plot([], [], [], lw=8, color="0.3")                      # bar between the hips
for leg in LEGS:
    leg["line"], = ax.plot([], [], [], "o-", lw=5, markersize=7, color=COLORS[leg["name"]])
    leg["status"] = fig.text(0.3, 0.08 if leg["name"] == "back" else 0.04, "", fontsize=9,
                             color=COLORS[leg["name"]], family="monospace")
title = ax.set_title("")

gait_buttons = RadioButtons(fig.add_axes([0.03, 0.02, 0.12, 0.1]), list(GAITS),
                            active=list(GAITS).index("push"))   # start with both legs pushing together
state = {"t": 0.0, "body_y": 0.0}

def update(_frame):
    gait = gait_buttons.value_selected
    state["t"] += DT
    state["body_y"] += body_speed(gait) * DT
    t, body_y = state["t"], state["body_y"]

    for leg, local, on_ground in leg_targets(t, gait):
        name = leg["name"]
        try:
            q = tuple(a + 0.0 for a in ik3(*local))   # + 0.0 turns -0.0 into 0.0 for display
        except ValueError as e:
            leg["line"].set_color("tab:gray")
            leg["status"].set_text(f"{name:>5}: {e}")
            continue
        xs, ys, zs = zip(*joint_positions(leg, body_y, *q))
        leg["line"].set_data_3d(xs, ys, zs)
        leg["line"].set_color(COLORS[name])
        phase = "stance" if on_ground else "swing "
        leg["status"].set_text(f"{name:>5} {phase}  q1 = %6.1f  q2 = %6.1f  q3 = %6.1f deg" % q)

    body_line.set_data_3d([0, 0], [body_y, body_y + SPACING], [HIP_HEIGHT, HIP_HEIGHT])
    centre = body_y + SPACING / 2
    ax.set_ylim(centre - VIEW, centre + VIEW)   # camera follows the body
    ticks = range(int((centre - VIEW) // 50 + 1) * 50, int(centre + VIEW), 50)
    ground.set_data_3d([0] * len(ticks), list(ticks), [0] * len(ticks))
    title.set_text(f"{gait} gait   body y = {body_y:.0f} mm   speed = {body_speed(gait):.0f} mm/s")
    return []

anim = FuncAnimation(fig, update, interval=DT * 1000, cache_frame_data=False)
plt.show()
