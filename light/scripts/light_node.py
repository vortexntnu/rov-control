#!/usr/bin/env python
import rospy
import Adafruit_BBIO.GPIO as GPIO
from vortex_msgs.msg import LightInput, Pwm

FULL_CYCLE = rospy.get_param('/pwm/counter/max')
GPIO_PIN_MAP = rospy.get_param('/light/gpio_pins')
PWM_PIN_MAP = rospy.get_param('/light/pwm_pins')
PWM_SCALING = 100


class LightNode(object):
    def __init__(self):
        rospy.init_node('light_node')
        self.sub = rospy.Subscriber('toggle_light', LightInput, self.callback, queue_size=10)
        self.pub_pwm = rospy.Publisher('pwm', Pwm, queue_size=10)
        for light_name in GPIO_PIN_MAP:
            GPIO.setup(GPIO_PIN_MAP[light_name], GPIO.OUT)

    def callback(self, msg):
        self.light_control(msg)

    def light_control(self, msg):
        light = msg.toggle_light
        rospy.loginfo("light: %s", light)
        if light in PWM_PIN_MAP:
            rospy.loginfo("iz pwm, try to fix.")
            pwm_msg = Pwm()
            pwm_msg.pins.append(PWM_PIN_MAP[light])
            pwm_msg.on.append(0)
            pwm_msg.off.append((FULL_CYCLE * msg.intensity) // PWM_SCALING)
            self.pub_pwm.publish(pwm_msg)
        elif light in GPIO_PIN_MAP:
            rospy.loginfo("iz gpio, try to set")
            if msg.intensity > 0:
                GPIO.output(GPIO_PIN_MAP[light], GPIO.HIGH)
            else:
                GPIO.output(GPIO_PIN_MAP[light], GPIO.LOW)


if __name__ == '__main__':
    try:
        light_node = LightNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
