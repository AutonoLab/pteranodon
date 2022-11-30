import cv2


def create_capture_gst(port=5600):
    """Create a capture object using Gstreamer
    Args:
        port (int, optional): Port to use. Defaults to 5600.
    Returns:
        cv2.VideoCapture: Capture object
    """
    args = f"udpsrc port={port}"
    args += "! application/x-rtp,payload=96,encoding-name=H264"
    args += "! rtpjitterbuffer mode=1"
    args += "! rtph264depay"
    args += "! decodebin"
    args += "! videoconvert"
    args += "! appsink"

    return cv2.VideoCapture(
        args,
        cv2.CAP_GSTREAMER,
    )
