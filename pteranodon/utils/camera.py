import cv2


def create_capture_gst(port=5600):
    """Create a capture object using Gstreamer
    Args:
        port (int, optional): Port to use. Defaults to 5600.
    Returns:
        cv2.VideoCapture: Capture object
    """
    return cv2.VideoCapture(f"udpsrc port={port} ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
