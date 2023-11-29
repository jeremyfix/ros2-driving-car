# Adapted from teleop_twist_keyboard which is released under BSD License

import sys
import threading
import rclpy
import driving_unity_interface.msg

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
    u    i    o
    j    k    l
    ,    ;    :

CTRL-C to quit

"""

moveBindings = {
    "u": (1.0, -0.43),
    "i": (1.0, 0.0),
    "o": (1.0, 0.43),
    "j": (0.0, -0.43),
    "k": (0.0, 0.0),
    "l": (0.0, 0.43),
    ",": (-1.0, -0.43),
    ";": (-1.0, 0.0),
    ":": (-1.0, 0.43),
}


def getKey(settings):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node("teleop_twist_keyboard")

    # parameters
    stamped = node.declare_parameter("stamped", False).value
    frame_id = node.declare_parameter("frame_id", "").value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    CommandMsg = driving_unity_interface.msg.CarCommand

    pub = node.create_publisher(CommandMsg, "command", 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    command_msg = CommandMsg()

    throttle = 0.0
    steering = 0.0
    try:
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                throttle = moveBindings[key][0]
                steering = moveBindings[key][1]
            else:
                print(key)

            if key == "\x03":
                break
            command_msg.throttle = throttle
            command_msg.steering = steering
            pub.publish(command_msg)

    except Exception as e:
        print(e)

    finally:
        command_msg.throttle = 0.0
        command_msg.steering = 0.0
        pub.publish(command_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == "__main__":
    main()
