import mpu6050
import time
import math
import asyncio
import moteus as m

# MPU6050 setup
mpu = mpu6050.mpu6050(0x68)

offsets = {
    "x": -1.006, 
    "y": 0.262, 
    "z": 1.438
}

# === PID Gains (tune these) ===
Kp = 40.0
Ki = 3.0
Kd = 1.5

# Sampling time (s)
Ts = 0.01  # 10 ms = 100 Hz

# Control state
integral = 0.0
prev_error = 0.0

# Torque clamp (based on your motor specs)
MAX_TORQUE = 2.0  # Nm

# === Read angle from accelerometer ===
def read_angle_x():
    accel = mpu.get_accel_data()
    calibrated = {axis: accel[axis] + offsets[axis] for axis in accel}
    try:
        angle_y = math.acos(calibrated["y"] / 9.81) - (math.pi / 2)
    except Exception as e:
        print("Angle calculation error:", e)
        angle_y = 0
    return angle_y

# === Async PID Control Loop ===
async def run():
    global integral, prev_error

    controller = m.Controller()
    print("Starting model-free PID control loop...")

    try:
        while True:
            # 1. Read current angle
            theta = read_angle_x()
            theta_ref = 0.0  # balanced reference

            # 2. Compute error and PID terms
            error = theta_ref - theta
            derivative = (error - prev_error) / Ts
            integral += error * Ts

            # 3. Compute control torque
            torque = Kp * error + Kd * derivative + Ki * integral

            # 4. Clamp & anti-windup
            if torque > MAX_TORQUE:
                torque = MAX_TORQUE
                integral -= error * Ts  # anti-windup correction
            elif torque < -MAX_TORQUE:
                torque = -MAX_TORQUE
                integral -= error * Ts

            prev_error = error

            # 5. Send torque command
            await controller.set_position(
                position = math.nan,
                velocity = math.nan,
                maximum_torque = math.nan,
                feedforward_torque = torque,
                query = False
            )

            print(f"Theta: {theta:.4f} rad | Torque: {torque:.4f} Nm")

            await asyncio.sleep(Ts)

    except KeyboardInterrupt:
        await controller.set_position(position=math.nan, velocity=0.0)
        print("Control loop stopped.")

# === MAIN EXECUTION ===
if __name__ == '__main__':
    asyncio.run(run())