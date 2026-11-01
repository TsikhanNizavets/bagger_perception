import cv2
import rclpy
import time
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLOE
from ultralytics.engine.results import Results
from message_filters import Subscriber, ApproximateTimeSynchronizer


class TargetDetection(Node):
    def __init__(self):
        super().__init__('target_detection')

        self.show_images = self.declare_parameter('show_images', False).value
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

        self.masked_depth_pub = self.create_publisher(
            Image,
            '/masked_depth',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,),
        )

    def image_sync_callback(self, colcor_msg, depth_msg):
        start = time.perf_counter()

        image = self.bridge.imgmsg_to_cv2(colcor_msg, desired_encoding='bgr8')

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # self.get_logger().info(f'Depth: {depth}\n')
        resulting_mask = np.full_like(depth, 0, dtype=np.uint8)
        masked_depth = np.full_like(depth, np.nan)

        result = self.model.predict(source=image, verbose=True)[0]

        # self.get_logger().info(f'Mask: {result.masks}\n')

        for mask in result.masks or []:
            # self.get_logger().info(f'Mask: {mask.data[0].size()}\n')
            # self.get_logger().info(f'Mask: {mask.data[1].size()}\n')
            # with np.printoptions(threshold=np.inf):
            #     self.get_logger().info(f'Mask: {result.masks.data[0].cpu().numpy() > 0.5}\n', throttle_duration_sec=20.0)
            # mask = result.masks.data[0].cpu().numpy() > 0.5
            # with np.printoptions(threshold=np.inf):
            #    self.get_logger().info(f'Mask bool: {np.array(mask.data[0].cpu().numpy(), dtype=np.float32)}', throttle_duration_sec=20.0)


            # if mask.shape != depth.shape:
            #     raise ValueError("Mask and depth must have the same dimensions")
            # self.get_logger().info(f'type of resutlich_mask: {type(resutling_mask)}, type of result.masks.data[0].cpu().numpy(): {type(result.masks.data[0].cpu().numpy())}\n')
            self.get_logger().info(f'type of resutlich_mask: {resulting_mask.dtype}, type of result.masks.data[0].cpu().numpy(): {result.masks.data[0].cpu().numpy().dtype}\n')

            resulting_mask = cv2.bitwise_or(mask.data[0].cpu().numpy(), resulting_mask, dst=resulting_mask, mask=mask.data[0].cpu().numpy())
            
        masked_depth = cv2.bitwise_and(depth, depth, mask=resulting_mask)

        output_msg = self.bridge.cv2_to_imgmsg(
            masked_depth,
            encoding=depth_msg.encoding,
        )
        output_msg.header = depth_msg.header

        self.masked_depth_pub.publish(output_msg)

        if self.show_images:
            cv2.imshow('YOLOE detections', result.plot())
            cv2.waitKey(1)
                    
        elapsed_ms = (time.perf_counter() - start) * 1000
        self.get_logger().info(f"Callback duration: {elapsed_ms:.1f} ms")


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_images:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
