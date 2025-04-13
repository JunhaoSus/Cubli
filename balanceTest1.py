import mpu6050
import time
import math
import numpy as np
import asyncio
import moteus as m  # Alias moteus as m for convenience
import control as ctrl  # pip install control

# Create MPU6050 object
mpu = mpu6050.mpu6050(0x68)

# Calibration offsets (update with your own computed values)
offsets = {
    "x": -1.006, 
    "y": 0.262, 
    "z": 1.438
}

# === CONTINUOUS-TIME TRANSFER FUNCTION DEFINITION ===
# G(s) = (-28.45*s^4 - 1043*s^3 - 7643*s^2 - 569*s) / (s^4 + 0.2449*s^3 - 43590*s^2 - 11.75*s)
numG = [-28.45, -1043, -7643, -569, 0]
denG = [1, 0.2449, -43590, -11.75, 0]
G_cont = ctrl.TransferFunction(numG, denG)

# Gain from tunable block (from MATLAB)
C_gain = 0.036725

# Combined (open-loop) transfer function: G_open(s) = C * G(s)
G_open = C_gain * G_cont

# === DISCRETIZATION ===
Ts = 0.01  # Sample time = 10 ms
G_d = ctrl.sample_system(G_open, Ts, method='tustin')

# Extract numerator and denominator coefficients for the discrete transfer function
num = np.squeeze(G_d.num)
den = np.squeeze(G_d.den)

print("Discrete Transfer Function Coefficients:")
print("Numerator (num):", num)
print("Denominator (den):", den)

# Initialize histories for the difference equation based on the number of coefficients
input_hist = [0] * len(num)
output_hist = [0] * (len(den) - 1)

# === SENSOR READING FUNCTION ===
def read_angle_x():
    """Reads accelerometer data and computes the x-axis angle in radians.
       (Here we use the 'y' axis of the sensor for our calculation; adjust as needed.)"""
    accel = mpu.get_accel_data()
    # Apply calibration offsets
    calibrated = {axis: accel[axis] + offsets[axis] for axis in accel}
    try:
        # Calculate y-axis angle from accelerometer data (convert from degrees to radians)
        # math.acos returns radians. Here we subtract pi/2 to shift the angle.
        angle_y = math.acos(calibrated["y"] / 9.81) - (math.pi/2)
    except Exception as e:
        print("Error in angle calculation:", e)
        angle_y = 0
    return angle_y

# === DIFFERENCE EQUATION IMPLEMENTATION ===
def compute_torque(theta_input):
    """
    Uses the difference equation representation of the discrete transfer function to compute output torque.
    theta_input is expected to be in radians.
    """
    # Update input history (most recent input first)
    input_hist.insert(0, theta_input)
    input_hist.pop()
    
    # Compute the new output using the discrete transfer function's difference equation:
    # y[n] = (num[0]*u[n] + num[1]*u[n-1] + ... ) - (den[1]*y[n-1] + den[2]*y[n-2] + ...)
    output = sum(n_coef * u for n_coef, u in zip(num, input_hist)) \
             - sum(d_coef * y for d_coef, y in zip(den[1:], output_hist))
    output /= den[0]
    
    # Update output history
    output_hist.insert(0, output)
    output_hist.pop()
    
    return output

# === ASYNC REAL-TIME CONTROL LOOP ===
async def run():
    # Create a moteus controller instance (specify id if needed)
    controller = m.Controller()
    
    # If using moteus.Stream for low-level commands, create it here
    stream = m.Stream(controller)
    
    print("Starting real-time torque control loop...")
    try:
        while True:
            # Read sensor angle in radians
            theta = read_angle_x()
            
            # Compute feedforward torque command using the difference equation
            torque = compute_torque(theta)
            
            # Print current sensor reading and computed torque
            print(f"Angle: {theta:.4f} rad | Torque: {torque:.4f} Nm")
            
            # Send the torque command to the motor via the moteus controller.
            # The "set_position" call allows you to send a feedforward torque while ignoring position/velocity.
            await controller.set_position(
                position = math.nan,          # Do not specify an absolute position
                velocity = math.nan,          # Do not control velocity here
                maximum_torque = math.nan,    # No explicit limit; you might want to add one
                feedforward_torque = torque,
                query = False              # No query needed on every iteration
            )
            
            # Wait for the next sample period using asyncio.sleep for non-blocking delay
            await asyncio.sleep(Ts)
            
    except KeyboardInterrupt:
        # On keyboard interrupt, safely command the motor to stop by setting velocity to 0.
        await controller.set_position(
            position = math.nan,
            velocity = 0,
            query = False
        )
        print("Control loop terminated. Velocity command set to zero.")

# === MAIN EXECUTION ===
if __name__ == '__main__':
    asyncio.run(run())
