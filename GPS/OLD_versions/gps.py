#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

GPS_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

def gps_publisher():
    rospy.init_node("gps_serial_publisher")
    gps_pub = rospy.Publisher("/fix", NavSatFix, queue_size=10)
    rate = rospy.Rate(5)

    ser = serial.Serial(GPS_PORT, BAUD_RATE, timeout=1)

    while not rospy.is_shutdown():
        line = ser.readline().decode('ascii', errors='replace')
        if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
            try:
                msg = pynmea2.parse(line)

                gps_msg = NavSatFix()
                gps_msg.header.stamp = rospy.Time.now()
                gps_msg.header.frame_id = "gps"
                gps_msg.latitude = msg.latitude
                gps_msg.longitude = msg.longitude
                gps_msg.altitude = msg.altitude if hasattr(msg, 'altitude') else 0.0

                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                gps_pub.publish(gps_msg)

                rospy.loginfo(f"Published: {msg.latitude}, {msg.longitude}")
            except pynmea2.ParseError:
                continue

        rate.sleep()

if __name__ == "__main__":
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass

