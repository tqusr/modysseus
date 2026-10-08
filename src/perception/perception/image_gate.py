"""On/off valve for the AprilTag detector's image stream.

apriltag_node subscribes for as long as it lives and detects on every frame,
which cost ~60% of a Jetson CPU core while nothing used the result. Only
`tag_dock_server` needs detections, and only for a few seconds at each pre-dock
pose, so the detector reads from this gate:

    camera / rectify_node  ->  image_gate  ->  apriltag_node

A closed gate holds no subscription to the source, so (rectify_node subscribes
lazily) nothing upstream processes frames either. Opening it subscribes and
forwards the serialized frames untouched, with no Python decode or encode.

Open and close it with the std_srvs/SetBool service `~/enable`
(`/image_gate/enable`). `max_open_sec` closes it again if the caller dies while
it is open.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class ImageGate(Node):
    """Forwards images from `source_topic` to `output_topic` while enabled.

    Parameters:
        source_topic: Image topic to forward from.
        output_topic: Image topic to publish on.
        max_open_sec: Seconds after which an open gate closes itself.
    """

    def __init__(self):
        super().__init__('image_gate')
        self.declare_parameter('source_topic', '/oak/rgb/image_rect')
        self.declare_parameter('output_topic', '/oak/rgb/image_gated')
        self.declare_parameter('max_open_sec', 30.0)
        self._source = self.get_parameter('source_topic').value
        self._max_open = float(self.get_parameter('max_open_sec').value)

        # Reliable, to match the detector's default-QoS subscription.
        self._publisher = self.create_publisher(
            Image, self.get_parameter('output_topic').value, 1)
        self._subscription = None
        self._timer = None
        self.create_service(SetBool, '~/enable', self._on_enable)
        self.get_logger().info(
            f'closed: {self._source} -> '
            f'{self.get_parameter("output_topic").value} on request')

    def _forward(self, serialized):
        """Republishes one serialized image."""
        self._publisher.publish(serialized)

    def _open(self):
        """Subscribes to the source and (re)arms the safety timeout."""
        if self._subscription is None:
            self._subscription = self.create_subscription(
                Image, self._source, self._forward, qos_profile_sensor_data,
                raw=True)
            self.get_logger().info('open')
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
        self._timer = self.create_timer(self._max_open, self._on_timeout)

    def _close(self):
        """Drops the source subscription and the safety timeout."""
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
            self.get_logger().info('closed')

    def _on_timeout(self):
        """Closes the gate after `max_open_sec` without a close request."""
        self.get_logger().warning(
            f'open for {self._max_open:.0f} s without being closed; closing')
        self._close()

    def _on_enable(self, request, response):
        """Opens or closes the gate per `request.data`."""
        if request.data:
            self._open()
        else:
            self._close()
        response.success = True
        return response


def main(args=None):
    """Runs the gate until shutdown."""
    rclpy.init(args=args)
    node = ImageGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
