import asyncio
import math as m
import moteus

async def run():
    # Create a controller instance (specify id if needed)
    controller = moteus.Controller()
    # Create a Stream for low-level commands
    stream = moteus.Stream(controller)
    
    constant_velocity = 20      # Desired constant velocity (adjust as needed)
    maximum_torque = 0.035       # Maximum allowed torque (in Nm)

    print("Spinning motor at constant velocity. Press Ctrl+C to stop.")
    try:
        while True:
            # Setting position to NaN tells the controller to use velocity mode.
            await controller.set_position(
                position=m.nan,          # Ignore absolute position
                velocity=constant_velocity,
                maximum_torque=maximum_torque,
                query=False              # No query needed in each loop iteration
            )
            await asyncio.sleep(0.05)
    except KeyboardInterrupt:
        # On keyboard interrupt, command the motor to stop by setting velocity to 0.
        await controller.set_position(
            position=m.nan,
            velocity=0,
            maximum_torque=maximum_torque,
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
