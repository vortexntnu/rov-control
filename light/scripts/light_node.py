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
        self.light_state = {}
        for key in GPIO_PIN_MAP:
            self.light_state[key] = False
            GPIO.setup(GPIO_PIN_MAP[key], GPIO.OUT)
        for key in PWM_PIN_MAP:
            self.light_state[key] = False

    def callback(self, msg):
        self.light_control(msg)

    def light_control(self, msg):
        for light in msg.toggle_light:
            if light in PWM_PIN_MAP:
                pwm_msg = Pwm()
                Pwm.pins[0] = PWM_PIN_MAP[light]
                Pwm.on[0] = 0
                if not self.light_state[light]:
                    Pwm.off = FULL_CYCLE
                    self.light_state[light] = not self.light_state[light]
                else:
                    Pwm.off = 0
                    self.light_state[light] = not self.light_state[light]
                self.pub_pwm.publish(pwm_msg)

            else if light in GPIO_PIN_MAP:
                if not self.light_state[light]:
                    GPIO.output(GPIO_PIN_MAP[light], GPIO.HIGH)
                    self.light_state[light] = not self.light_state[light]
                else:
                    GPIO.output(GPIO_PIN_MAP[light] ,GPIO.LOW)
                    self.light_state[light] = not self.light_state[light]


if __name__ == '__main__':
    try:
        light_node = LightNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass