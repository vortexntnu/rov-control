
class LightNode(object):
    def __init__(self):
        pass

if __name__ == '__main__':
    try:
        light_node = LightNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass