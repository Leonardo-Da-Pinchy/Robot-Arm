
import cv2
import numpy as np
import pyrealsense2 as rs
from pyapriltags import Detector

TAG_SIZE = 0.05  # Tag size in meters (5 cm)
AXIS_LENGTH = TAG_SIZE / 2

detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    refine_edges=1
)

# Initialize RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(
    rs.stream.color, 640, 480, rs.format.bgr8, 30
)

started = False

try:
    profile = pipeline.start(config)
    started = True

    # Get actual camera intrinsics
    stream = profile.get_stream(
        rs.stream.color
    ).as_video_stream_profile()

    intr = stream.get_intrinsics()

    fx, fy = intr.fx, intr.fy
    cx, cy = intr.ppx, intr.ppy

    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)

    camera_params = (fx, fy, cx, cy)

    # Undistort Brown-Conrady color frames if needed
    use_undistort = intr.model in (
        rs.distortion.brown_conrady,
        rs.distortion.modified_brown_conrady
    )
    dist = np.array(intr.coeffs, dtype=np.float64)
    dist_coeffs = np.zeros(5, dtype=np.float64)

    print("RealSense camera started")

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        if use_undistort:
            frame = cv2.undistort(
                frame, camera_matrix, dist
            )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Grayscale background with colored axes
        display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Detect tags and estimate 3D pose
        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=TAG_SIZE
        )

        for tag in tags:
            R = tag.pose_R
            t = tag.pose_t

            if R is None or t is None:
                continue

            # Convert pose into OpenCV format
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape(3, 1)

            # Draw colored 3D axes
            cv2.drawFrameAxes(
                display,
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
                AXIS_LENGTH,
                2
            )

            # Draw tag border
            corners = tag.corners.astype(np.int32)
            cv2.polylines(
                display, [corners], True, (255, 255, 255), 1
            )

            # Calculate horizontal surface angle
            normal = R[:, 2]
            angle = np.degrees(
                np.arctan2(normal[0], normal[2])
            )

            center = tag.center.astype(int)

            # Display tag ID and angle
            cv2.putText(
                display,
                f"ID {tag.tag_id}: {angle:.1f} deg",
                (int(center[0]), int(center[1]) - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        cv2.imshow("AprilTag 3D Pose", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    if started:
        pipeline.stop()
    cv2.destroyAllWindows()

