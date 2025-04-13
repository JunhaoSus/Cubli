import mpu6050
import time
import math
import numpy as np
import moteus

# Create MPU6050 object
mpu = mpu6050.mpu6050(0x68)

# Calibration offsets (update with yours)
offsets = {
    "x": -1.006, 
    "y": 0.262, 
    "z": 1.438
}

# === DISCRETIZED TRANSFER FUNCTION COEFFICIENTS ===
# From MATLAB: G(s) * C, then sampled with Ts = 0.01s
# These would normally be computed with `control.sample_system()` but are hardcoded here

# Define the continuous-time transfer function from the image
# G(s) = (-28.45*s^4 - 1043*s^3 - 7643*s^2 - 569*s) / (s^4 + 0.2449*s^3 - 43590*s^2 - 11.75*s)

numG = [-28.45, -1043, -7643, -569, 0]
denG = [1, 0.2449, -43590, -11.75, 0]
G = ctrl.TransferFunction(numG, denG)

# Gain C = 0.036725 from tunable block
C = 0.036725
G_open = C * G

# Initialize input/output history
input_hist = [0] * len(num)
output_hist = [0] * (len(den) - 1)

Ts = 0.01  # Sample time = 10 ms

async def run():
    # Create a controller instance (specify id if needed)
    controller = moteus.Controller()
    # Create a Stream for low-level commands
    stream = moteus.Stream(controller)

    # === READ SENSOR ===
    def read_angle_x():
        accel = mpu.get_accel_data()
        # Apply offsets
        calibrated = {axis: accel[axis] + offsets[axis] for axis in accel}
        # Calculate X angle from accelerometer (in degrees)
        try:
            angle_y = math.radians(math.acos(calibrated["y"] / 9.81)) - (math.pi/2)
        except:
            angle_y = 0  # Fallback on error
        return angle_y

    # === DIFFERENCE EQUATION IMPLEMENTATION ===
    def compute_torque(theta_input):
        # Shift and update input history
        input_hist.insert(0, theta_input)
        input_hist.pop()

        # Difference equation calculation
        output = sum(n * u for n, u in zip(num, input_hist)) \
            - sum(d * y for d, y in zip(den[1:], output_hist))

        output /= den[0]

        # Update output history
        output_hist.insert(0, output)
        output_hist.pop()

        return output

    # === MAIN LOOP ===
    print("Starting real-time torque control loop...")
    try:
        while True:
            angle = read_angle_x()
            torque = compute_torque(angle)

            # Print (replace this with motor control output)
            print(f"Angle X: {angle:.2f} rad | Torque: {torque:.4f} Nm")

            await controller.set_position(
                    position= m.nan,          # Ignore absolute position
                    velocity= m.nan,
                    maximum_torque= m.nan,
                    feedforward_torque = torque,
                    query=False              # No query needed in each loop iteration
                )
            await asyncio.sleep(Ts)

            time.sleep(Ts)

    except KeyboardInterrupt:
        # On keyboard interrupt, command the motor to stop by setting velocity to 0.
        await controller.set_position(
            position=m.nan,
            velocity=0,
            query=False
        )
        print("Velocity command set to zero.")
        
if __name__ == '__main__':
    asyncio.run(run())