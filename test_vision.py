#!/usr/bin/env python3
"""
ROS2 Car Fun - Vision Test Script
视觉处理测试脚本

Test script for all vision nodes
"""

import sys
import time
import threading
import argparse
import numpy as np

# 使用之前的模拟环境
import test_sensors


def patch_vision_imports():
    """修补视觉相关导入"""
    # 先修补基础ROS2导入
    test_sensors.patch_ros2_imports()

    # 添加视觉相关的消息类型
    class MockImage:
        def __init__(self):
            self.header = test_sensors.MockHeader()
            self.height = 480
            self.width = 640
            self.encoding = 'bgr8'
            self.data = b''

    class MockCompressedImage:
        def __init__(self):
            self.header = test_sensors.MockHeader()
            self.format = 'jpeg'
            self.data = b''

    # 更新sensor_msgs模块
    import sys
    if 'sensor_msgs.msg' in sys.modules:
        sys.modules['sensor_msgs.msg'].Image = MockImage
        sys.modules['sensor_msgs.msg'].CompressedImage = MockCompressedImage

    # 确保geometry_msgs.msg有Point类
    if 'geometry_msgs.msg' in sys.modules:
        if not hasattr(sys.modules['geometry_msgs.msg'], 'Point'):
            class MockPoint:
                def __init__(self):
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
            sys.modules['geometry_msgs.msg'].Point = MockPoint
    
    # 添加cv_bridge模拟
    class MockCvBridge:
        def cv2_to_imgmsg(self, cv_image, encoding='bgr8'):
            class MockImageMsg:
                def __init__(self):
                    self.header = test_sensors.MockHeader()
                    self.height = cv_image.shape[0] if len(cv_image.shape) > 1 else 1
                    self.width = cv_image.shape[1] if len(cv_image.shape) > 1 else len(cv_image)
                    self.encoding = encoding
                    self.data = cv_image.tobytes() if hasattr(cv_image, 'tobytes') else b''
            return MockImageMsg()
        
        def imgmsg_to_cv2(self, img_msg, desired_encoding='bgr8'):
            # 返回模拟图像
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    # 模拟cv2
    class MockCV2:
        # 常量
        CAP_PROP_FRAME_WIDTH = 3
        CAP_PROP_FRAME_HEIGHT = 4
        CAP_PROP_FPS = 5
        CAP_PROP_AUTO_EXPOSURE = 21
        CAP_PROP_BRIGHTNESS = 10
        CAP_PROP_CONTRAST = 11
        CAP_PROP_SATURATION = 12
        IMWRITE_JPEG_QUALITY = 1
        COLOR_BGR2GRAY = 6
        COLOR_BGR2HSV = 40
        RETR_EXTERNAL = 0
        CHAIN_APPROX_SIMPLE = 2
        MORPH_OPEN = 2
        MORPH_CLOSE = 3
        FONT_HERSHEY_SIMPLEX = 0
        
        @staticmethod
        def VideoCapture(index):
            class MockVideoCapture:
                def isOpened(self): return True
                def set(self, prop, value): return True
                def read(self): return True, np.zeros((480, 640, 3), dtype=np.uint8)
                def release(self): pass
            return MockVideoCapture()
        
        @staticmethod
        def cvtColor(image, code):
            if len(image.shape) == 3:
                return np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
            return image
        
        @staticmethod
        def GaussianBlur(image, ksize, sigmaX):
            return image
        
        @staticmethod
        def inRange(image, lower, upper):
            return np.zeros(image.shape[:2], dtype=np.uint8)
        
        @staticmethod
        def morphologyEx(image, op, kernel):
            return image
        
        @staticmethod
        def findContours(image, mode, method):
            # 返回空轮廓列表
            return [], None
        
        @staticmethod
        def contourArea(contour):
            return 100
        
        @staticmethod
        def moments(contour):
            return {'m00': 1, 'm10': 50, 'm01': 50}
        
        @staticmethod
        def boundingRect(contour):
            return (10, 10, 50, 50)
        
        @staticmethod
        def rectangle(image, pt1, pt2, color, thickness):
            return image
        
        @staticmethod
        def circle(image, center, radius, color, thickness):
            return image
        
        @staticmethod
        def putText(image, text, org, fontFace, fontScale, color, thickness):
            return image
        
        @staticmethod
        def line(image, pt1, pt2, color, thickness):
            return image
        
        @staticmethod
        def polylines(image, pts, isClosed, color, thickness):
            return image
        
        @staticmethod
        def drawContours(image, contours, contourIdx, color, thickness):
            return image
        
        @staticmethod
        def equalizeHist(image):
            return image
        
        @staticmethod
        def Canny(image, threshold1, threshold2):
            return np.zeros(image.shape[:2], dtype=np.uint8)
        
        @staticmethod
        def arcLength(contour, closed):
            return 100
        
        @staticmethod
        def approxPolyDP(contour, epsilon, closed):
            return np.array([[10, 10], [60, 10], [60, 60], [10, 60]])
        
        @staticmethod
        def convexHull(contour):
            return contour
        
        @staticmethod
        def imencode(ext, image, params=None):
            return True, np.array([1, 2, 3], dtype=np.uint8)
        
        # 级联分类器
        class CascadeClassifier:
            def __init__(self, filename):
                self.filename = filename
            
            def empty(self):
                return False
            
            def detectMultiScale(self, image, scaleFactor=1.1, minNeighbors=3, 
                               minSize=(30, 30), maxSize=(300, 300)):
                # 返回模拟人脸检测结果
                return np.array([[100, 100, 80, 80], [200, 150, 60, 60]])
        
        # QR码检测器
        class QRCodeDetector:
            def detectAndDecode(self, image):
                # 返回模拟QR码检测结果
                data = "TEST_QR_CODE"
                points = np.array([[[100, 100], [200, 100], [200, 200], [100, 200]]])
                return data, points, None
        
        # 数据路径
        class data:
            haarcascades = "/usr/share/opencv4/haarcascades/"
    
    # 注册模拟模块
    sys.modules['cv2'] = MockCV2()
    sys.modules['cv_bridge'] = type(sys)('cv_bridge')
    sys.modules['cv_bridge'].CvBridge = MockCvBridge
    
    # 模拟numpy（如果需要）
    if 'numpy' not in sys.modules:
        sys.modules['numpy'] = np


def test_camera_node():
    """测试摄像头节点"""
    print("\n=== Testing Camera Node ===")
    
    try:
        from ros2_car_fun.vision.camera_node import CameraNode
        
        node = CameraNode()
        print("✅ Camera node created successfully")
        
        # 测试摄像头信息
        info = node.get_camera_info()
        print(f"Camera info: {info}")
        
        # 测试云台控制
        if node.enable_servo_control:
            print("Testing servo control...")
            node.reset_camera_position()
            
            # 模拟云台命令
            class MockFloat32:
                def __init__(self):
                    self.data = 0.0

            pan_msg = MockFloat32()
            pan_msg.data = 120.0
            node.pan_callback(pan_msg)

            tilt_msg = MockFloat32()
            tilt_msg.data = 60.0
            node.tilt_callback(tilt_msg)
            
            print(f"Pan: {node.current_pan}, Tilt: {node.current_tilt}")
        
        # 清理
        node.cleanup()
        print("✅ Camera node test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Camera node test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_color_detection():
    """测试颜色检测"""
    print("\n=== Testing Color Detection ===")
    
    try:
        from ros2_car_fun.vision.color_detection import ColorDetectionNode
        
        node = ColorDetectionNode()
        print("✅ Color detection node created successfully")
        
        # 测试颜色设置
        node.set_target_color('blue')
        
        # 测试自定义颜色
        node.add_custom_color('pink', [(150, 50, 50), (170, 255, 255)])
        
        # 获取统计信息
        stats = node.get_detection_statistics()
        print(f"Detection statistics: {stats}")
        
        print("✅ Color detection test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Color detection test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_face_tracking():
    """测试人脸跟踪"""
    print("\n=== Testing Face Tracking ===")
    
    try:
        from ros2_car_fun.vision.face_tracking import FaceTrackingNode
        
        node = FaceTrackingNode()
        print("✅ Face tracking node created successfully")
        
        # 测试统计信息
        stats = node.get_face_statistics()
        print(f"Face statistics: {stats}")
        
        # 测试重置跟踪
        node.reset_tracking()
        
        print("✅ Face tracking test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Face tracking test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_qr_detection():
    """测试二维码检测"""
    print("\n=== Testing QR Detection ===")
    
    try:
        from ros2_car_fun.vision.qr_detection import QRDetectionNode
        
        node = QRDetectionNode()
        print("✅ QR detection node created successfully")
        
        # 测试统计信息
        stats = node.get_qr_statistics()
        print(f"QR statistics: {stats}")
        
        print("✅ QR detection test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ QR detection test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_object_detection():
    """测试物体检测"""
    print("\n=== Testing Object Detection ===")
    
    try:
        from ros2_car_fun.vision.object_detection import ObjectDetectionNode
        
        node = ObjectDetectionNode()
        print("✅ Object detection node created successfully")
        
        # 测试统计信息
        stats = node.get_detection_statistics()
        print(f"Object statistics: {stats}")
        
        print("✅ Object detection test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Object detection test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_vision_integration():
    """测试视觉集成"""
    print("\n=== Testing Vision Integration ===")
    
    try:
        # 测试多个视觉节点协同工作
        from ros2_car_fun.vision.camera_node import CameraNode
        from ros2_car_fun.vision.color_detection import ColorDetectionNode
        
        camera = CameraNode()
        color_detector = ColorDetectionNode()
        
        print("✅ Multiple vision nodes created")
        
        # 模拟图像处理流程
        print("Simulating image processing pipeline...")
        
        # 获取当前帧（仿真）
        frame = camera.get_current_frame()
        if frame is not None:
            print(f"Got frame: {frame.shape}")
        else:
            print("No frame available (simulation mode)")
        
        # 清理
        camera.cleanup()
        
        print("✅ Vision integration test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Vision integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_vision_tests():
    """运行所有视觉测试"""
    print("👁️  ROS2 Car Fun - Vision Tests")
    print("=" * 50)
    
    tests = [
        ("Camera Node", test_camera_node),
        ("Color Detection", test_color_detection),
        ("Face Tracking", test_face_tracking),
        ("QR Detection", test_qr_detection),
        ("Object Detection", test_object_detection),
        ("Vision Integration", test_vision_integration),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n🧪 Running: {test_name}")
        print("-" * 40)
        
        try:
            result = test_func()
            results.append(result)
            
            if result:
                print(f"✅ {test_name} PASSED")
            else:
                print(f"❌ {test_name} FAILED")
                
        except Exception as e:
            print(f"❌ {test_name} CRASHED: {e}")
            results.append(False)
    
    # 总结
    print("\n" + "=" * 50)
    passed = sum(results)
    total = len(results)
    
    print(f"📊 Vision Test Results: {passed}/{total} passed")
    
    if passed == total:
        print("🎉 All vision tests passed!")
        print("\n✨ Vision system is ready!")
    else:
        print("⚠️  Some vision tests failed")
        print("🔧 Please check the error messages above")
    
    return passed == total


def interactive_vision_test():
    """交互式视觉测试"""
    print("\n=== Interactive Vision Test ===")
    print("Commands:")
    print("  1 - Test camera node")
    print("  2 - Test color detection")
    print("  3 - Test face tracking")
    print("  4 - Test QR detection")
    print("  5 - Test object detection")
    print("  6 - Test vision integration")
    print("  a - Test all")
    print("  q - Quit")
    
    while True:
        try:
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == '1':
                test_camera_node()
            elif cmd == '2':
                test_color_detection()
            elif cmd == '3':
                test_face_tracking()
            elif cmd == '4':
                test_qr_detection()
            elif cmd == '5':
                test_object_detection()
            elif cmd == '6':
                test_vision_integration()
            elif cmd == 'a':
                run_all_vision_tests()
            elif cmd == 'q':
                break
            else:
                print("Unknown command")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("Interactive test ended")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Vision Test Script')
    parser.add_argument('--test', choices=['camera', 'color', 'face', 'qr', 'object', 'integration', 'all'],
                       default='all', help='Test to run')
    parser.add_argument('--interactive', action='store_true', help='Interactive mode')
    
    args = parser.parse_args()
    
    print("👁️  ROS2 Car Fun - Vision Test")
    print("=" * 50)
    
    # 修补导入
    patch_vision_imports()
    
    try:
        if args.interactive:
            interactive_vision_test()
        else:
            if args.test == 'camera':
                test_camera_node()
            elif args.test == 'color':
                test_color_detection()
            elif args.test == 'face':
                test_face_tracking()
            elif args.test == 'qr':
                test_qr_detection()
            elif args.test == 'object':
                test_object_detection()
            elif args.test == 'integration':
                test_vision_integration()
            elif args.test == 'all':
                run_all_vision_tests()
        
    except Exception as e:
        print(f"\n❌ Vision test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
