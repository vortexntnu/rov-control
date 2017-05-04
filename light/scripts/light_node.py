#!/usr/bin/env python
import rospy
import Adafruit_BBIO.GPIO as GPIO
from vortex_msgs.msg import LightInput
from vortex_msgs.msg import Pwm

PWM_LIGHT_PIN = 9 # Placeholder
FULL_CYCLE = 4096
BT_LIGHT_PIN = "P8_10" # Placeholder
RAMAN_LIGHT_PIN = "P8_12" # Placeholder

class LightNode(object):
    def __init__(self):
        rospy.init_node('light_node')
        self.sub = rospy.Subscriber('light', LightInput, self.callback, queue_size=10)
        self.pub_pwm = rospy.Publisher('pwm', Pwm, queue_size=10)
        GPIO.setup(BT_LIGHT_PIN, GPIO.OUT)
        GPIO.setup(RAMAN_LIGHT_PIN, GPIO.OUT)
        self.pin_map = {"bluetooth":BT_LIGHT_PIN,"raman":RAMAN_LIGHT_PIN}

    def callback(self, msg):
        if len(msg.light) == len(msg.on):
            self.light_control(msg)

    def light_control(self, msg):
        for i in range(len(msg.light)):
            if msg.light[i] == "front_pwm":
                pwm_msg = Pwm
                Pwm.pins[0] = PWM_LIGHT_PIN
                Pwm.on[0] = 0
                if msg.on[i]:
                    Pwm.off = FULL_CYCLE
                else:
                    Pwm.off = 0
                self.pub_pwm.publish(pwm_msg)

            else if msg.light[i] == "bluetooth" or msg.light[i] == "raman":
                if msg.on[i]:
                    GPIO.output(pin_map[msg.light[i]],GPIO.HIGH)
                else:
                    GPIO.output(pin_map[msg.light[i]],GPIO.LOW)


if __name__ == '__main__':
    try:
        light_node = LightNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass