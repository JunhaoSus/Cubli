import asyncio
import math as m
import moteus
import mpu6050
import time

mpu6050 = mpu6050.mpu6050(0x68)


async def run():
    # Create a controller instance (specify id if needed)
    controller = moteus.Controller()
    # Create a Stream for low-level commands
    stream = moteus.Stream(controller)

    constant_velocity = 2      # Desired constant velocity (adjust as needed)
    maximum_torque = 0.5      # Maximum allowed torque (in Nm)

    print("Spinning motor at constant velocity. Press Ctrl+C to stop.")
    try:
        while True:
            accelerometer_data = mpu6050.get_accel_data()
            # Setting position to NaN tells the controller to use velocity mode.
            await controller.set_position(
                position=m.nan,          # Ignore absolute position
                velocity=constant_velocity*accelerometer_data['y'],
                maximum_torque=m.nan,
                query=False              # No query needed in each loop iteration
            )
            # print("Accelerometer data:", accelerometer_data)
            # print(constant_velocity*accelerometer_data)
            await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        # On keyboard interrupt, command the motor to stop by setting velocity to 0.
        await controller.set_position(
            position=m.nan,
            velocity=0,
            # maximum_torque=m.nan,
            query=False
        )
        print("Velocity command set to zero.")
    finally:
        try:
            # Disable the motor driver by sending the stop commands.
            await stream.write_message(b"tel stop")
            await stream.flush_read()
            await stream.command(b"d stop")
            print("Motor driver disabled.")
        except Exception as e:
            print(f"Error while disabling motor driver: {e}")

if __name__ == '__main__':
    asyncio.run(run())
