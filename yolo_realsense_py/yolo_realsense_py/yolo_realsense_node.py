import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import numpy as np
from geometry_msgs.msg import Point
from detectionresult.msg import DetectionResult

class YOLORealsenseNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')

        # 检查是否有 GPU
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')

        # 加载 YOLOv11 模型
        self.model = YOLO('/home/evan/yolodc_ws/src/yolodc_ws/model/yolov8n.pt')  # 使用 YOLO 类加载
        self.model.to(self.device)  # 移动到指定设备（CPU/GPU）
        self.model.eval()  # 设置为推理模式

        # 初始化 CV Bridge
        self.bridge = CvBridge()

        # 订阅 RealSense 的 RGB 和深度图像
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10)

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        # 发布检测结果
        self.detection_publisher = self.create_publisher(DetectionResult, '/detection_result', 10)

        # 存储最新的 RGB 和深度图像
        self.latest_rgb_image = None
        self.latest_depth_image = None

    def rgb_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb_image = cv_image
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f'Failed to convert RGB image: {e}')

    def depth_callback(self, msg):
        # 将 ROS 深度图像消息转换为 OpenCV 格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = cv_image
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    def process_frame(self):
        if self.latest_rgb_image is None or self.latest_depth_image is None:
            return

        # 将图像转换为模型输入格式
        input_image = cv2.resize(self.latest_rgb_image, (640, 640))  # 调整图像尺寸
        input_image = input_image.transpose(2, 0, 1)  # HWC to CHW
        input_image = np.ascontiguousarray(input_image, dtype=np.float32)  # 转换为连续数组
        input_image /= 255.0  # 归一化

        # 添加 batch 维度并移动到 GPU
        input_tensor = torch.from_numpy(input_image).unsqueeze(0).to(self.device)

        # 推理
        with torch.no_grad():
            results = self.model(input_tensor)

        # 解析检测结果
        for result in results:
        # 确保 result 是 ultralytics 的 Results 对象
            if hasattr(result, "boxes") and result.boxes is not None:
                detections = result.boxes.data.cpu().numpy()  # 提取检测框数据
    
        # 遍历每个检测框
                for detection in detections:
                    x1, y1, x2, y2, confidence, class_id = map(float, detection[:6])  # 确保转换为 float 类型

                    if confidence > 0.8:  
                        # 计算中心点
                        center_x = int((x1 + x2) / 2) - 320
                        center_y = int((y1 + y2) / 2) - 320 

                        # 获取深度值
                        depth = self.latest_depth_image[center_y, center_x] / 1000.0  # 转换为米
                        """ if depth == 0:
                            self.get_logger().warn('Depth value is zero.')
                            continue """
                        # 发布自定义消息
                        if depth != 0:
                            detection_msg = DetectionResult()
                            detection_msg.class_id = int(class_id)
                            detection_msg.confidence = float(confidence)
                            detection_msg.center = Point(x=float(center_x), y=float(center_y), z=float(depth))
                            self.detection_publisher.publish(detection_msg)
                            # self.get_logger().info(f'Detected class {int(class_id)} ({center_x}, {center_y}, {depth})')

def main(args=None):
    rclpy.init(args=args)
    node = YOLORealsenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()