import cv2
import numpy as np
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# RealSenseカメラのデバイス番号
device_number = 0
capture = cv2.VideoCapture(device_number)

while True:
    color_frame = pipeline.wait_for_frames().get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    ret, frame = capture.read()
    if ret:
        cv2.imshow("RealSense Camera", color_image)
        cv2.imshow("OpenCV Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

pipeline.stop()
capture.release()
cv2.destroyAllWindows()
