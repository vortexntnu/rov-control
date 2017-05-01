import rospy
import serial
import Adafruit_BBIO.UART as UART
from std_msgs.msg import String
import binascii

class Hc05InterfaceNode(object):
    def __init__(self):
        rospy.init_node('bluetooth_node')
        self.port = "/dev/tty04"
        self.init_serial()
        self.init_publisher()
        self.read_bluetooth_data()

    def init_serial(self):
        UART.setup("UART4")
        self.ser = serial.Serial(port=self.port,
                        baudrate=9600,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1)
        if not self.ser.is_open():
            rospy.logwarn("Unable to open %s", self.ser)

    def init_publisher(self):
        self.pub_bluetooth = rospy.Publisher(
            'sensors/bluetooth/data',
            String,
            queue_size=10)

    def read_bluetooth_data(self):
        while not rospy.is_shutdown():
            BT_msg = _self._readline()
            if len(BT_msg) > 0:
                BT_msg = binascii.b2a_uu(BT_msg)
                self.pub_bluetooth.publish(BT_msg)
            rospy.Rate(10).sleep()

    def _readline(self):
        eol = b'\r'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.ser.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

if __name__ == '__main__':
    try:
        bluetooth_node = Hc05InterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass