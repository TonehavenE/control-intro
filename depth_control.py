from pymavlink import mavutil
import sys
import signal
from pid import PID
import numpy as np


def set_rc_channel_pwm(mav, channel_id, pwm=1500):
    """Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    mav.mav.rc_channels_override_send(
        mav.target_system,  # target_system
        mav.target_component,  # target_component
        *rc_channel_values
    )


def set_vertical_power(mav, power=0):
    """Set vertical power
    Args:
        power (int, optional): Vertical power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range. Clipping...")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 3, 1500 + power * 5)


def press_to_depth(pressure):
    """Convert pressure to depth
    Args:
        pressure (float): Pressure in hPa
    Returns:
        float: Depth in water in meters
    """
    rho = 1029  # density of fresh water in kg/m^3
    g = 9.81  # gravity in m/s^2
    pressure_at_sea_level = 1013.25  # pressure at sea level in hPa
    # multiply by 100 to convert hPa to Pa
    return (pressure - pressure_at_sea_level) * 100 / (rho * g)


def main():
    mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

    # catch CTRL+C
    def signal_handler(sig, frame):
        print("CTRL+C pressed. Disarming")
        mav.arducopter_disarm()
        mav.motors_disarmed_wait()
        print("Disarmed")
        sys.exit(0)

    # catch CTRL+C
    signal.signal(signal.SIGINT, signal_handler)

    # wait for the heartbeat message to find the system ID
    mav.wait_heartbeat()

    # arm the vehicle
    print("Arming")
    mav.arducopter_arm()
    mav.motors_armed_wait()
    print("Armed")

    # set mode to MANUAL
    print("Setting mode to MANUAL")
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        19,  # Manual mode
    )
    set_vertical_power(
            mav, 0
        )
    print("Mode set to MANUAL")

    # ask user for depth
    desired_depth = float(input("Enter target depth: "))
    pid = PID(40, 0.00, -10.0, 80)

    while True:
        # get pressure from the vehicle
        msg = mav.recv_match(type="SCALED_PRESSURE2", blocking=True)
        press_abs = msg.press_abs  # in hPa

        # calculate depth
        current_depth = press_to_depth(press_abs)

        print("Depth: ", current_depth)

        # calculate error
        error = desired_depth - current_depth
        print("Error: ", error)

        output = pid.update(error)
        print("Output: ", output)

        # set vertical power
        set_vertical_power(
            mav, -output
        )  # negative because of the direction of the thruster


if __name__ == "__main__":
    main()
