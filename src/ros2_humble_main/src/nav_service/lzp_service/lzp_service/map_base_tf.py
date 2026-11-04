import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped


class TFToPose(Node):
    def __init__(self):
        super().__init__('tf_to_pose')
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().info('TF to Pose node started (debug mode)')
        
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0).to_msg())
        self.listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.timer = self.create_timer(0.2, self.tick)
        
        self.tick_counter = 0
        self.start_time = self.get_clock().now()

    def tick(self):
        self.tick_counter += 1
        current_time = self.get_clock().now()
        
        try:
            # 调试：显示可用帧
            # if self.tick_counter == 1:
            #     # self.get_logger().debug("=== First tick ===")
            #     # 检查是否有任何帧
            #     try:
            #         all_frames = self.tf_buffer.all_frames_as_string()
            #         if all_frames:
            #             self.get_logger().info(f"Frames in buffer:\n{all_frames}")
            #     except:
            #         pass
            
            # 方法A：尝试使用零时间
            # self.get_logger().debug(f"Tick {self.tick_counter}: Trying lookup_transform with zero time")
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),  # 零时间
                timeout=Duration(seconds=0.05)
            )
            
            # 打印TF的时间戳
            tf_time = tf.header.stamp
            # self.get_logger().debug(f"Got transform with stamp: sec={tf_time.sec}, nanosec={tf_time.nanosec}")
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = current_time.to_msg()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation.x = tf.transform.rotation.x
            pose.pose.orientation.y = tf.transform.rotation.y
            pose.pose.orientation.z = tf.transform.rotation.z
            pose.pose.orientation.w = tf.transform.rotation.w

            self.pub.publish(pose)
            
            # if self.tick_counter % 5 == 0:  # 每5次发布一次日志
            #     self.get_logger().info(f'[SUCCESS] Published pose: [{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}]')
                
        except TransformException as e:
            # 尝试诊断问题
            error_msg = str(e)
            self.get_logger().debug(f'TransformException: {error_msg}')
            
            # 检查是否有map和base_link帧
            if self.tick_counter % 20 == 0:
                try:
                    # 检查帧是否存在
                    frames = self.tf_buffer.all_frames_as_string()
                    self.get_logger().info(f"Current frames in buffer:\n{frames}")
                except:
                    pass
                    
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TFToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
