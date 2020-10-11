#!/usr/bin/env python

import rospy
import time
import board
import busio
import adafruit_gps

from sensor_msgs.msg import NavSatFix
import serial

def gps():
    uart = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=10)
    gps = adafruit_gps.GPS(uart, debug=False)
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,1000')
    last_print = time.monotonic()
    pub = rospy.Publisher('NavSatFix', NavSatFix, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        gps.update()
        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
        if not gps.has_fix:
            print('Waiting for fix...')
            continue
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now() 
        msg.header.frame_id = 'gps'
        msg.latitude = gps.latitude
        msg.longitude = gps.longitude
        msg.altitude = gps.altitude_m
        msg.status.status = gps.fix_quality
        msg.status.service = 1
        pub.publish(msg)


if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass

