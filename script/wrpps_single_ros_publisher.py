import rospy

from wrpps_driver.wrpps_single import WrPPSSingleDriver
from wrpps_driver.msg import WrPPSRange


class WrPPSROSPublisher(WrPPSSingleDriver):
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, frame_id='wrpps_sensor'):
        super().__init__(port=port, baudrate=baudrate)

        self.frame_id = frame_id

        # Create ROS publishers for intensity and TOF data
        self.range_pub = rospy.Publisher('~range', WrPPSRange, queue_size=10)

    def _handle_data(self, data, data_time):
        """
        This method is called whenever a new line of serial data is parsed.
        It publishes the parsed intensity and TOF values to their respective topics.
        """
        intensity = data.get('intensity')
        tof = data.get('tof')
        sec_ = int(data_time)
        nanosec_ = int((data_time - sec_)*1e9)

        msg = WrPPSRange()
        msg.header.stamp = rospy.Time(sec_, nanosec_)
        msg.header.frame_id = self.frame_id
        if intensity is not None:
            msg.intensity = intensity
        if tof is not None:
            msg.distance = float(tof) / 1000.0

        self.range_pub.publish(msg)


def main():
    rospy.init_node('wrpps_ros_publisher')

    # Get parameters from the ROS parameter server
    port = rospy.get_param('~port', '/dev/ttyACM0')
    baudrate = rospy.get_param('~baudrate', 115200)
    frame_id = rospy.get_param('~frame_id', 'wrpps_sensor')

    # Create and start the publisher
    driver = WrPPSROSPublisher(port=port, baudrate=baudrate, frame_id=frame_id)

    try:
        driver.start_reading()
        rospy.loginfo("WrPPS ROS Publisher started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        driver.close()
        rospy.loginfo("WrPPS ROS Publisher stopped.")


if __name__ == '__main__':
    main()
