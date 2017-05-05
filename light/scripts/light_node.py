#!/usr/bin/env python
import rospy
import Adafruit_BBIO.GPIO as GPIO
from vortex_msgs.msg import LightInput
from vortex_msgs.msg import Pwm

FULL_CYCLE = 4096
GPIO_PIN_MAP = rospy.get_param('/light/gpio_pins')
PWM_PIN_MAP = rospy.get_param('/light/pwm_pins')

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
        for index, light in enumerate(msg.toggle_light):
            if light in PWM_PIN_MAP:
                pwm_msg = Pwm()
                Pwm.pins[0] = PWM_PIN_MAP[light]
                Pwm.on[0] = 0
                Pwm.off = (FULL_CYCLE * msg.intensity[index]) // 100
                self.pub_pwm.publish(pwm_msg)

            else if light in GPIO_PIN_MAP:
                if msg.intensity[index] > 0:
                    GPIO.output(GPIO_PIN_MAP[light], GPIO.HIGH)
                else:
                    GPIO.output(GPIO_PIN_MAP[light] ,GPIO.LOW)


if __name__ == '__main__':
    try:
        light_node = LightNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass