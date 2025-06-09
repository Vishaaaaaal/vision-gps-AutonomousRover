#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import termios
import tty

def getKey():
    """Function to capture single keypress"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node("keyboard_control")
    pub = rospy.Publisher("nano_control", String, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    
    print("🚗 Rover Control with Keyboard")
    print("Use Arrow Keys to Move | 'q' to Quit")
    print("[⬆️] Forward | [⬇️] Backward | [⬅️] Left | [➡️] Right | [s] Stop")

    while not rospy.is_shutdown():
        key = getKey()
        
        if key == '\x1b':  # Detect arrow key press
            key = getKey()
            if key == '[':
                key = getKey()
                if key == 'A':
                    pub.publish("forward")
                    print("Moving Forward")
                elif key == 'B':
                    pub.publish("backward")
                    print("Moving Backward")
                elif key == 'C':
                    pub.publish("right")
                    print("Turning Right")
                elif key == 'D':
                    pub.publish("left")
                    print("Turning Left")
        elif key == 's':
            pub.publish("stop")
            print("Stopping")
        elif key == 'q':
            print("Exiting...")
            break
        
        rate.sleep()

if __name__ == "__main__":
    main()

