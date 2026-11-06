"""Live sliders for the leg servos. Needs the servo_cal firmware flashed with CAL_STEP = CAL_SLIDER.

Close idf.py monitor first (it holds the port), then:
    sim/.venv/bin/python sim/servo_sliders.py [port]

One slider per connected servo, in raw servo units (0..1000 = 0..240 deg). Each label shows
the position the servo reports back. The sliders start at the servos' current positions.
Home sends every servo to its 0 deg position (the .zero values in main/joints.c).
"""
import glob
import sys
import time

import matplotlib.pyplot as plt
import serial
from matplotlib.widgets import Button, Slider

SEND_PERIOD_MS = 40  # how often a drag gets sent to the servo
HOME_MS = 1500       # Home moves slowly so the leg doesn't snap there at full speed


def pick_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.SLAB*") + glob.glob("/dev/ttyUSB*")
    if not ports:
        sys.exit("No USB serial port found. Pass one: servo_sliders.py /dev/cu.xxx")
    return ports[0]


def open_board(port):
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, 0
    ser.dtr = ser.rts = False  # don't reset the ESP32 when the port opens
    ser.open()
    return ser


def list_joints(ser):
    """Ask the board for its servos. Returns [(id, pos, zero, name)]."""
    for _ in range(10):
        ser.reset_input_buffer()
        ser.write(b"L\n")
        joints, buf, t_end = [], b"", time.time() + 0.5
        while time.time() < t_end:
            buf += ser.read(256)
        for line in buf.decode(errors="ignore").splitlines():
            parts = line.strip().split(" ", 4)
            # the board also prints its list at boot, so the same servo can show up twice
            if len(parts) == 5 and parts[0] == "J" and int(parts[1]) not in {j[0] for j in joints}:
                joints.append((int(parts[1]), int(parts[2]), int(parts[3]), parts[4]))
        if joints:
            return joints
    sys.exit("Board didn't answer. Is servo_cal flashed with CAL_SLIDER, and idf.py monitor closed?")


port = pick_port()
ser = open_board(port)
joints = list_joints(ser)
print(f"{port}: " + ", ".join(f"{name} (ID {i}, zero {z})" for i, _, z, name in joints))

rows = len(joints) + 1  # +1 for the Home button
fig = plt.figure(figsize=(8, 1.5 + 0.8 * len(joints)))
fig.canvas.manager.set_window_title("Servo sliders")
sliders, pending = {}, {}  # pending: id -> (pos, move ms)
move_ms = None              # set while Home is moving the sliders
for k, (sid, pos, zero, name) in enumerate(joints):
    ax = fig.add_axes([0.25, 1 - (k + 1) / (rows + 1), 0.6, 0.5 / (rows + 1)])
    s = Slider(ax, f"{name}  ID {sid}", 0, 1000, valinit=max(0, pos), valstep=1)
    s.on_changed(lambda v, sid=sid: pending.__setitem__(sid, (int(v), move_ms)))
    sliders[sid] = s


def home(_event):
    global move_ms
    move_ms = HOME_MS
    for sid, _, zero, _ in joints:
        sliders[sid].set_val(zero)
    move_ms = None


home_button = Button(fig.add_axes([0.4, 0.5 / (rows + 1), 0.2, 0.6 / (rows + 1)]), "Home (0 deg)")
home_button.on_clicked(home)

rx = b""


def tick():
    global rx
    for sid, (pos, ms) in list(pending.items()):
        ser.write((f"M {sid} {pos} {ms}" if ms else f"M {sid} {pos}").encode() + b"\n")
    pending.clear()

    rx += ser.read(4096)
    *lines, rx = rx.split(b"\n")
    for line in lines:
        parts = line.decode(errors="ignore").split()
        if not parts or parts[0] != "P":
            continue
        for item in parts[1:]:
            sid, _, pos = item.partition("=")
            if sid.isdigit() and int(sid) in sliders:
                sliders[int(sid)].valtext.set_text(f"{int(sliders[int(sid)].val)}  (at {pos})")
    fig.canvas.draw_idle()


timer = fig.canvas.new_timer(interval=SEND_PERIOD_MS)
timer.add_callback(tick)
timer.start()
plt.show()
ser.close()
