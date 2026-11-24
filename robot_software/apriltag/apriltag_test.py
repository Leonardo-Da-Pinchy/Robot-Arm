
import cv2
import numpy as np
import pyrealsense2 as rs
from pyapriltags import Detector

# ==========================================
# CONFIGURATION
# ==========================================

WIDTH = 640
HEIGHT = 480
FPS = 30

TAG_SIZE = 0.0238125  # Physical tag size in meters
AXIS_LENGTH = TAG_SIZE / 2
DISTANCE_OFFSET = 0.0086 # physical offset from optical center to front of camera
NUM_TAGS = 3 # Maximum number of tags we want to detect

# Camera position relative to robot origin
CAMERA_X = 0.1016  # 4 inches
CAMERA_Y = 0.0
CAMERA_Z = 0.0     # TODO: Measure camera height

# Camera orientation
# Heading is measured CCW from robot +X
CAMERA_HEADING = 120.0
CAMERA_PITCH = 0.0  # Positive if pointing upward

# ==========================================
# CAMERA-TO-ROBOT TRANSFORMATION
# ==========================================

theta = np.radians(CAMERA_HEADING - 90.0)
pitch = np.radians(CAMERA_PITCH)

# Convert optical frame into robot frame
R_0 = np.array([
    [1,  0, 0],
    [0,  0, 1],
    [0, -1, 0]
])

# Horizontal rotation
R_z = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0, 0, 1]
])

# Vertical camera tilt
R_x = np.array([
    [1, 0, 0],
    [0, np.cos(pitch), -np.sin(pitch)],
    [0, np.sin(pitch),  np.cos(pitch)]
])

# Combined rotation
R_robot_camera = R_z @ R_x @ R_0

# Translation from robot origin to camera
t_robot_camera = np.array([
    CAMERA_X,
    CAMERA_Y,
    CAMERA_Z
])


def camera_to_robot(position):
    return R_robot_camera @ position + t_robot_camera


# ==========================================
# INITIALIZE APRILTAG DETECTOR
# ==========================================

detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    refine_edges=1
)

# ==========================================
# INITIALIZE REALSENSE CAMERA
# ==========================================

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    WIDTH,
    HEIGHT,
    rs.format.bgr8,
    FPS
)

started = False

try:
    profile = pipeline.start(config)
    started = True

    color_profile = profile.get_stream(
        rs.stream.color
    ).as_video_stream_profile()

    intr = color_profile.get_intrinsics()

    camera_matrix = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1]
    ], dtype=np.float64)

    camera_params = (
        intr.fx,
        intr.fy,
        intr.ppx,
        intr.ppy
    )

    # Lens distortion correction
    use_undistort = intr.model in (
        rs.distortion.brown_conrady,
        rs.distortion.modified_brown_conrady
    )

    distortion = np.array(intr.coeffs)
    zero_distortion = np.zeros(5)

    print("Camera started")

    # ======================================
    # DETECTION LOOP
    # ======================================

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        if use_undistort:
            frame = cv2.undistort(
                frame, camera_matrix, distortion
            )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display = frame.copy()

        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=TAG_SIZE
        )

        for i in range(len(tags)):
            tag = tags[i]
            if i < NUM_TAGS:
                if tag.pose_t is None or tag.pose_R is None:
                    continue

                # Position relative to camera (meters)
                x_cam, y_cam, z_cam = tag.pose_t.flatten()
                
                z_corrected = z_cam - DISTANCE_OFFSET
                
                p_camera = [x_cam, y_cam, z_corrected]

                # Distance from camera
                distance_camera = np.linalg.norm(p_camera)

                # Position relative to robot
                p_robot = camera_to_robot(p_camera)

                x_robot, y_robot, z_robot = p_robot

                # Distance from robot origin
                distance_robot = np.linalg.norm(p_robot)

                # Horizontal distance from robot origin
                distance_xy = np.hypot(x_robot, y_robot)

                # Tag orientation relative to robot
                R_robot_tag = R_robot_camera @ tag.pose_R

                # Outward-facing surface normal
                normal = -R_robot_tag[:, 2]

                # Surface normal angle measured from robot +X
                surface_angle = np.degrees(
                    np.arctan2(normal[1], normal[0])
                )

                # Draw 3D coordinate axes
                rvec, _ = cv2.Rodrigues(tag.pose_R)

                cv2.drawFrameAxes(
                    display,
                    camera_matrix,
                    zero_distortion,
                    rvec,
                    tag.pose_t.reshape(3, 1),
                    AXIS_LENGTH,
                    2
                )

                # Draw tag outline
                corners = tag.corners.astype(np.int32)

                cv2.polylines(
                    display, [corners], True,
                    (0, 255, 0), 2
                )

                center = tag.center.astype(int)

                # Information displayed on camera feed
                lines = [
                    f"ID: {tag.tag_id}",
                    f"Cam dist: {distance_camera * 100:.1f} cm",
                    f"Robot dist: {distance_robot * 100:.1f} cm",
                    f"X: {x_robot * 100:.1f} cm",
                    f"Y: {y_robot * 100:.1f} cm",
                    f"Z: {z_robot * 100:.1f} cm",
                    f"Surface: {surface_angle:.1f} deg"
                ]

                text_x = max(0, min(center[0] + 15, WIDTH - 210))
                text_y = max(20, min(center[1] - 40, HEIGHT - 135))

                for i, line in enumerate(lines):
                    cv2.putText(
                        display,
                        line,
                        (text_x, text_y + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 255),
                        1
                    )

                print(
                    f"ID: {tag.tag_id} | "
                    f"Camera distance: {distance_camera:.4f} m | "
                    f"Robot XYZ: {p_robot.round(4)} | "
                    f"XY distance: {distance_xy:.4f} m | "
                    f"Surface angle: {surface_angle:.1f} deg"
                )

        cv2.imshow("AprilTag Robot Localization", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    if started:
        pipeline.stop()

    cv2.destroyAllWindows()
