import mpu6050
import time

# Create a new mpu6050 object
mpu = mpu6050.mpu6050(0x68)

# Example: Pre-determined calibration offsets (update these with your computed values)
offsets = {
    "x": -1.006320761,   # Replace 9.5 with your computed average for the x axis.
    "y": 0.262073514,   # Replace 0.3 with your computed average for the y axis.
    "z": 1.438033123 # Replace -0.2 with your computed average for the z axis.
}

def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu.get_accel_data()
    
    # Apply the calibration offsets
    calibrated_accel = {
        axis: accelerometer_data[axis] + offsets[axis]
        for axis in accelerometer_data
    }
    
    # Read gyroscope values
    gyroscope_data = mpu.get_gyro_data()

    # Read temperature
    temperature = mpu.get_temp()

    return calibrated_accel, gyroscope_data, temperature

# Continuously read the sensor data
while True:
    calibrated_accel, gyroscope_data, temperature = read_sensor_data()

    # Print the sensor data
    print("Calibrated Accelerometer data:", calibrated_accel)
    print("Gyroscope data:", gyroscope_data)
    print("Temperature:", temperature)

    # Wait for 1 second
    time.sleep(1)
