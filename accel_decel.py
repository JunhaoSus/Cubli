import asyncio
import math as m
import moteus

async def run():
    # Create the controller instance.
    controller = moteus.Controller()
    # Create a Stream for low-level commands.
    stream = moteus.Stream(controller)
    
    # Define parameters.
    maximum_torque = 0.10  # Maximum allowed torque in Nm.
    cycle_time = 0.05      # Loop time interval in seconds.
    
    try:
        print("Starting control loop. Press Ctrl+C to stop.")
        
        while True:
            # Phase 1: Accelerate forward (high velocity) over 1.0 second.
            phase_duration = 1
            target_velocity_forward = 20  # High forward velocity (rad/s)
            start_time = asyncio.get_event_loop().time()
            while (t := asyncio.get_event_loop().time() - start_time) < phase_duration:
                desired_velocity = (t / phase_duration) * target_velocity_forward
                await controller.set_position(
                    position=m.nan,
                    velocity=desired_velocity,
                    maximum_torque=maximum_torque,
                    query=False
                )
                await asyncio.sleep(cycle_time)
            
            # Phase 2: Quick deceleration over 0.5 seconds.
            phase_duration = 0.5
            start_time = asyncio.get_event_loop().time()
            # Decelerate from +1.0 to 0 rad/s quickly.
            while (t := asyncio.get_event_loop().time() - start_time) < phase_duration:
                desired_velocity = target_velocity_forward * (1 - (t / phase_duration))
                await controller.set_position(
                    position=m.nan,
                    velocity=desired_velocity,
                    maximum_torque=maximum_torque,
                    query=False
                )
                await asyncio.sleep(cycle_time)
            
            # Phase 3: Accelerate in the negative direction slowly over 2.0 seconds.
            phase_duration = 2.0
            target_velocity_reverse = -0.5  # Slower reverse velocity (rad/s)
            start_time = asyncio.get_event_loop().time()
            while (t := asyncio.get_event_loop().time() - start_time) < phase_duration:
                # Ramp from 0 to -0.5 rad/s gradually.
                desired_velocity = (t / phase_duration) * target_velocity_reverse
                await controller.set_position(
                    position=m.nan,
                    velocity=desired_velocity,
                    maximum_torque=maximum_torque,
                    query=False
                )
                await asyncio.sleep(cycle_time)
            
            # Phase 4: Gradual deceleration from reverse over 2.0 seconds.
            phase_duration = 2.0
            start_time = asyncio.get_event_loop().time()
            while (t := asyncio.get_event_loop().time() - start_time) < phase_duration:
                # Ramp from -0.5 to 0 rad/s.
                desired_velocity = target_velocity_reverse * (1 - (t / phase_duration))
                await controller.set_position(
                    position=m.nan,
                    velocity=desired_velocity,
                    maximum_torque=maximum_torque,
                    query=False
                )
                await asyncio.sleep(cycle_time)
                
    except KeyboardInterrupt:
        # On Ctrl+C, command the motor to stop.
        await controller.set_position(
            position=m.nan,
            velocity=0,
            maximum_torque=maximum_torque,
            query=False
        )
        print("Motor velocity set to zero. Exiting control loop.")
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
