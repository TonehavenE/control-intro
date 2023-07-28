from pymavlink import mavutil
import sys
import signal
from pid import PID
import numpy as np


def set_rc_channel_pwm(mav, channel_id, pwm=1500):
    """Set RC channel pwm value
    Args:a
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


def set_rotation_power(mav, power=0, go_forward=False):
    """Set rotation power
    Args:
        power (int, optional): Power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range.")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 4, 1500 + power * 5)
    if go_forward:
        set_rc_channel_pwm(mav, 6, 1500 + 20 * 5)   

def map_angle(h):
    return np.arctan2(np.sin(h), np.cos(h))

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

    desired_heading_deg = float(input("Enter target heading: "))

    # arm the vehicle
    print("Arming")
    mav.arducopter_arm()
    mav.motors_armed_wait()
    set_rc_channel_pwm(mav, 4, 1500 + 0 * 5) # yaw
    set_rc_channel_pwm(mav, 5, 1500 + 0 * 5) # forward
    set_rc_channel_pwm(mav, 6, 1500 + 0) # lateral
    print("Armed")

    # set mode to MANUAL
    print("Setting mode to MANUAL")
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        19,  # Manual mode
    )
    set_rotation_power(mav, 0)

    print("Mode set to MANUAL")

    # ask user for depth

    # TODO: convert heading to radians
    desired_heading = np.radians(desired_heading_deg)

    pid = PID(35, 0.05, -10, 100)

    while True:
        # get yaw from the vehicle
        msg = mav.recv_match(type="ATTITUDE", blocking=True)
        yaw = msg.yaw
        yaw_rate = msg.yawspeed

        # print("Heading: ", np.rad2deg(yaw))

        error = desired_heading - yaw
        print(f"Error: {np.rad2deg(error)}")
        go_forward = False
        if abs(360 - np.rad2deg(error)) % 360 < 10:
            print("going forward!")
            go_forward = True

        error %= (np.pi * 2)
        if (error > (np.pi / 2)) and (error < np.pi):
            error = 1
        elif (error < (3 * np.pi) / 2) and (error > np.pi):
            error = -1
        else:
            error = np.sin(error)

        # print("Angle Diff: ", angle_diff)

        output = pid.update(error, error_derivative=yaw_rate)
        print("Output: ", output)

        # set vertical power
        set_rotation_power(mav, output, go_forward)


if __name__ == "__main__":
    main()
