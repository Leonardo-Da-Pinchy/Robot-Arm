
import cv2
import numpy as np
import pyrealsense2 as rs
from pyapriltags import Detector

# Initialize AprilTag detector
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1
)

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    640, 480,
    rs.format.bgr8,
    30
)

try:
    pipeline.start(config)
    print("Camera started")

    while True:
        # Get camera frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert RealSense frame into OpenCV image
        frame = np.asanyarray(color_frame.get_data())

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        tags = detector.detect(gray)

        for tag in tags:
            # Get tag ID and coordinates
            tag_id = tag.tag_id
            center = tuple(tag.center.astype(int))
            corners = tag.corners.astype(int)

            # Draw tag outline
            cv2.polylines(
                frame,
                [corners],
                True,
                (0, 255, 0),
                2
            )

            # Draw tag center
            cv2.circle(
                frame,
                center,
                5,
                (0, 0, 255),
                -1
            )

            # Display tag ID
            cv2.putText(
                frame,
                f"ID: {tag_id}",
                (center[0] + 10, center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            print(f"Tag {tag_id}: center={center}")

        # Display feed
        cv2.imshow("AprilTag Detection", frame)

        # Press q to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

