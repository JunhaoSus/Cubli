import numpy as np
import math
import asyncio
import mpu6050
import moteus as m
import control as ctrl
import time

# === SENSOR SETUP ===
mpu = mpu6050.mpu6050(0x68)
offsets = {"x": -1.006, "y": 0.262, "z": 1.438}

# === PLANT TRANSFER FUNCTION ===
# G(s) = -18.97 / (s^3 + 0.2449*s^2 - 43590*s - 11.75)
numG = [-18.97]
denG = [1, 0.2449, -43590, -11.75]
G = ctrl.tf(numG, denG)

# Convert to state-space (controllable canonical form)
sys_ss = ctrl.tf2ss(numG, denG)
A = sys_ss.A
B = sys_ss.B
C = sys_ss.C
D = sys_ss.D

# === POLE PLACEMENT ===
zeta = 0.7
wn = 4.76
target_poles = [complex(-zeta*wn, wn*np.sqrt(1 - zeta**2)),
                complex(-zeta*wn, -wn*np.sqrt(1 - zeta**2)),
                -20]
K = ctrl.place(A, B, target_poles)

# Closed-loop state-space system
A_cl = A - B @ K
C_flipped = -C  # Correct for original negative gain
sys_cl = ctrl.ss(A_cl, B, C_flipped, D)

# === ZEROS AND POLES OF CLOSED-LOOP SYSTEM ===
sys_tf = ctrl.ss2tf(A_cl, B, C_flipped, D)
zeros = ctrl.zero(sys_tf)
poles = ctrl.pole(sys_tf)
print("Closed-loop Zeros:", zeros)
print("Closed-loop Poles:", poles)

# === STATE and IMU INITIALIZATION ===
x = np.zeros((A.shape[0], 1))
Ts = 0.015  # 100 Hz

# === SENSOR FUNCTION (with averaging 8 samples) ===
def read_angle_x():
    angles = []
    for _ in range(8):
        accel = mpu.get_accel_data()
        calibrated = {axis: accel[axis] + offsets[axis] for axis in accel}
        try:
            angle_y = math.acos(calibrated["y"] / 9.81) - (math.pi/2)
        except:
            angle_y = 0
        angles.append(angle_y)
        time.sleep(0.001)  # ~1kHz IMU sampling
    return sum(angles) / len(angles)

# === REAL-TIME CONTROL LOOP ===
async def run():
    global x
    controller = m.Controller()
    stream = m.Stream(controller)
    print("Starting pole-placement control loop...")

    try:
        while True:
            theta = read_angle_x()
            x[0, 0] = theta
            u = -K @ x
            u = u.item()

            # Torque limiter (saturate to +/- 1 Nm)
            u = max(min(u, 1), -1)

            print(f"Angle: {theta:f} rad | Torque: {-u:f} Nm")

            await controller.set_position(
                position = math.nan,
                velocity = math.nan,
                maximum_torque = math.nan,
                feedforward_torque = -u,
                query = False
            )
            await asyncio.sleep(Ts)

    except KeyboardInterrupt:
        await controller.set_position(position = math.nan, velocity = 0, query = False)
        print("Control loop terminated. Velocity set to zero.")

if __name__ == '__main__':
    asyncio.run(run())
