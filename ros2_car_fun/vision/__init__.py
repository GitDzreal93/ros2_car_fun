"""
ROS2 Car Fun - Vision Module
智能小车视觉处理模块

This module provides computer vision capabilities for the smart car,
including camera control, color detection, face tracking, QR code recognition,
and object detection.
"""

from .camera_node import CameraNode
from .color_detection import ColorDetectionNode
from .face_tracking import FaceTrackingNode
from .qr_detection import QRDetectionNode
from .object_detection import ObjectDetectionNode

__all__ = [
    'CameraNode',
    'ColorDetectionNode',
    'FaceTrackingNode', 
    'QRDetectionNode',
    'ObjectDetectionNode'
]
