#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# 🎮 Joystick Mapping
STEER_AXIS = 2   # Right Stick X (Steering)
RT_AXIS = 4      # Right Trigger (Throttle)
STOP_BUTTON = 3  # X Button for Emergency Stop

prev_throttle = 0  # Track previous throttle state
prev_steering = 0  # Track previous steering position

def joy_callback(msg):
    global prev_throttle, prev_steering

    # 🛞 Steering Logic (Right Stick X)
    steering = msg.axes[STEER_AXIS] * 100  # Convert -1 to 1 into -100% to 100%
    rt_value = msg.axes[RT_AXIS]  # RT axis raw value (-1 to 1)
    stop = msg.buttons[STOP_BUTTON]  # X Button for Emergency Stop

    command = None

    # 🏎️ Map RT (Throttle) from 0 to 850
    throttle_input = int(((1 - rt_value) / 2) * 850)

    if stop:
        command = "stop"
        rospy.loginfo("🛑 EMERGENCY STOP")

    elif abs(steering) > 5:  # Only send commands if significant movement
        direction = "left" if steering > 0 else "right"
        command = f"{direction} {int(abs(steering))}%"
        prev_steering = steering
        rospy.loginfo(f"⬅️➡️ Turning {direction.capitalize()} {int(abs(steering))}%")

    elif prev_steering != 0:  # Joystick Centered → Return to Center
        command = "center"
        prev_steering = 0
        rospy.loginfo("🔄 Returning Steering to Center")

    # 🏎️ Throttle Handling - Gradual Progression and Return to Idle
    if throttle_input > 0 and prev_throttle == 0:
        command = "throttle"
        rospy.loginfo("🚀 Accelerating - Stepper Moves Forward")

    elif throttle_input == 0 and prev_throttle > 0:
        command = "reverse_throttle"
        rospy.loginfo("⬅️ Returning to Idle Position - Stepper Moves Backward")

    prev_throttle = throttle_input  # Store previous throttle state

    if command:
        pub.publish(command)

# 🔧 Initialize ROS Node
rospy.init_node("joystick_control")
pub = rospy.Publisher("nano_control", String, queue_size=10)
rospy.Subscriber("joy", Joy, joy_callback)

rospy.loginfo("🎮 Joystick Control Active")
rospy.spin()

