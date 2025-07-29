
#This is a sample simulation for the PID controller with three IR sensors used to trace a line
import matplotlib.pyplot as plt
import random
from controller import PIDController

# Convert 3 sensor readings to a line position error
def get_error_from_sensors(sensors):
    # sensors = [Left, Center, Right], each 0 or 1
    weights = [1, 0, -1]  # Left = +1, Center = 0, Right = -1
    active = sum(sensors)
    
    if active == 0:
        return None  # no line detected
    else:
        #multiply the weight and the sensor values and sum the results
        weighted_sum = sum(w * s for w, s in zip(weights, sensors))
        return weighted_sum / active

def simulate_pid_with_3_sensors(Kp, Ki, Kd):
    pid = PIDController(Kp, Ki, Kd)

    time = [0]
    dt = 0.1 #time steps delta t
    total_time = 20

    # Start with robot slightly off center
    current_error = 1
    errors = [current_error]
    control_signals = []

    last_known_error = current_error

    for t in range(int(total_time / dt)):
        # Simulate sensor readings based on current error
        # Add randomness to simulate sensor behavior
        if current_error > 0.5:
            sensors = [1, 0, 0]
        elif current_error > 0.1:
            sensors = [1, 1, 0]
        elif current_error < -0.5:
            sensors = [0, 0, 1]
        elif current_error < -0.1:
            sensors = [0, 1, 1]
        else:
            sensors = [0, 1, 0]  # centered

        # Simulate noise
        if random.random() < 0.05:
            sensors = [0, 0, 0]  # momentary dropout

        error = get_error_from_sensors(sensors)

        if error is None:
            error = last_known_error
        else:
            last_known_error = error

        control = pid.update(error, dt)
        control_signals.append(control)

        # Update current error based on control
        # Assume control improves alignment
        current_error -= control * dt
        errors.append(current_error)
        time.append(time[-1] + dt) #add the time step dt to the last time value

    # Plot results
    plt.figure(figsize=(10, 5))
    plt.plot(time, errors, label='Line Position Error')
    plt.plot(time[:-1], control_signals, label='PID Output (Motor Control)')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('PID Simulation with 3-Sensor Line Follower')
    plt.grid(True)
    plt.legend()
    plt.show()

# Run simulation
simulate_pid_with_3_sensors(Kp=1.0, Ki=0.1, Kd=0.2)