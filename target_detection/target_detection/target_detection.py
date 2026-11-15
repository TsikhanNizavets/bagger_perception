import cv2
import rclpy
import time
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLOE
from message_filters import Subscriber, ApproximateTimeSynchronizer
from bagger_interfaces.msg import MaskedDepthArray


class TargetDetection(Node):
    def __init__(self):
        super().__init__('target_detection')

        self.show_yoloe_detections = self.declare_parameter('show_yoloe_detections', False).value
        self.bridge = CvBridge()
        self.model = YOLOE('yoloe-11s-seg.pt')
        self.model.set_classes(['red cube'])

        self.color_image_sub = Subscriber(
            self,
            Image,
            '/rqbd_camera/image',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,),
        )

        self.depth_image_sub = Subscriber(
            self,
            Image,
            '/rqbd_camera/depth_image',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,),
        )

        queue_size = 10
        max_delay = 0.05
        self.image_sync = ApproximateTimeSynchronizer([self.color_image_sub, self.depth_image_sub],
                                                      queue_size=queue_size, slop=max_delay)
        self.image_sync.registerCallback(self.image_sync_callback)

        self.target_depth_pub = self.create_publisher(
            MaskedDepthArray,
            '/target_detection/masked_depth_array',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,),
        )

    def image_sync_callback(self, colcor_msg, depth_msg):
        start = time.perf_counter()

        image = self.bridge.imgmsg_to_cv2(colcor_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        masked_depth_array = MaskedDepthArray()

        result = self.model.predict(source=image, verbose=False)[0]

        for mask in result.masks or []:
            masked_depth_array.masked_depth.append(self.bridge.cv2_to_imgmsg(cv2.bitwise_or(depth, depth, dst=np.full_like(depth, np.nan), mask=mask.data[0].cpu().numpy())))
            masked_depth_array.masked_depth[-1].header = depth_msg.header

        self.target_depth_pub.publish(masked_depth_array)

        if self.show_yoloe_detections:
            cv2.imshow('YOLOE detections', result.plot())
            cv2.waitKey(1)
                    
        elapsed_ms = (time.perf_counter() - start) * 1000
        self.get_logger().debug(f"Callback duration: {elapsed_ms:.1f} ms")


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_yoloe_detections:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
