import cv2
import numpy as np
import pyrealsense2 as rs
from pyapriltags import Detector

WIDTH = 640
HEIGHT = 480
FPS = 30
TAG_SIZE = 0.0238125  # 15/16 inches, meters
AXIS_LENGTH = TAG_SIZE / 2
DISTANCE_OFFSET = 0.0086  # Empirical forward offset, meters
APPLY_OFFSET = True  # Revalidate offset after rectification


def rotation_to_euler(R):
    # R = Rz(roll) @ Ry(yaw) @ Rx(pitch)
    # Camera axes: X right, Y down, Z forward.
    sy = np.hypot(R[0, 0], R[1, 0])
    if sy >= 1e-6:
        pitch = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(-R[2, 0], sy)
        roll = np.arctan2(R[1, 0], R[0, 0])
    else:
        pitch = np.arctan2(-R[1, 2], R[1, 1])
        yaw = np.arctan2(-R[2, 0], sy)
        roll = 0.0
    return np.degrees([roll, pitch, yaw])


def rectification_maps(intr, camera_matrix, distortion):
    size = (intr.width, intr.height)
    if intr.model == rs.distortion.brown_conrady:
        return cv2.initUndistortRectifyMap(
            camera_matrix, distortion, None, camera_matrix,
            size, cv2.CV_32FC1
        )

    if intr.model == rs.distortion.inverse_brown_conrady:
        # Deproject raw pixels onto a unit-depth plane using RealSense.
        # This is a geometric ray calculation, NOT a depth sensor reading.
        u, v = np.meshgrid(
            np.arange(intr.width, dtype=np.float32),
            np.arange(intr.height, dtype=np.float32)
        )
        dx = np.empty_like(u)
        dy = np.empty_like(v)
        for row in range(intr.height):
            for col in range(intr.width):
                x, y, _ = rs.rs2_deproject_pixel_to_point(
                    intr, [float(col), float(row)], 1.0
                )
                dx[row, col] = intr.fx * x + intr.ppx - col
                dy[row, col] = intr.fy * y + intr.ppy - row

        # Invert the raw-to-rectified displacement field to obtain
        # the rectified-to-raw sampling coordinates cv2.remap needs.
        mx, my = u.copy(), v.copy()
        for _ in range(30):
            sx = cv2.remap(dx, mx, my, cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_REPLICATE)
            sy = cv2.remap(dy, mx, my, cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_REPLICATE)
            mx, my = u - sx, v - sy
        sx = cv2.remap(dx, mx, my, cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_REPLICATE)
        sy = cv2.remap(dy, mx, my, cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_REPLICATE)
        residual = np.hypot(mx + sx - u, my + sy - v)
        valid = ((mx >= 0) & (mx <= intr.width - 1) &
                 (my >= 0) & (my <= intr.height - 1))
        if not np.any(valid) or np.max(residual[valid]) > 0.5:
            raise RuntimeError("Rectification map failed to converge.")
        print(f"Map inversion residual: {np.max(residual[valid]):.3f} px")
        mx[~valid] = -1
        my[~valid] = -1
        return mx, my

    if intr.model == rs.distortion.modified_brown_conrady:
        mx = np.empty((intr.height, intr.width), dtype=np.float32)
        my = np.empty_like(mx)
        for row in range(intr.height):
            for col in range(intr.width):
                ray = [(col - intr.ppx) / intr.fx,
                       (row - intr.ppy) / intr.fy, 1.0]
                mx[row, col], my[row, col] = rs.rs2_project_point_to_pixel(
                    intr, ray
                )
        return mx, my

    raise RuntimeError(f"Unsupported distortion model: {intr.model}")


def main():
    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        refine_edges=1
    )

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS
    )
    started = False

    try:
        profile = pipeline.start(config)
        started = True
        intr = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()

        camera_matrix = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ], dtype=np.float64)
        camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)
        distortion = np.array(intr.coeffs, dtype=np.float64)
        zero_distortion = np.zeros(5, dtype=np.float64)

        # Use the SDK's own distortion model to build a rectified image.
        maps = None
        print(f"Distortion model: {intr.model}")
        print(f"Distortion coefficients: {distortion}")
        if intr.model != rs.distortion.none and np.any(distortion):
            print("Building rectification maps...")
            maps = rectification_maps(intr, camera_matrix, distortion)

        print("Camera started. Press q to quit.")
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            if maps is not None:
                frame = cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            display = frame.copy()

            tags = detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=camera_params, tag_size=TAG_SIZE
            )

            for i in range(len(tags)):
                if i > 8:
                    continue

                tag = tags[i]
                if tag.pose_t is None or tag.pose_R is None:
                    continue
                R = tag.pose_R
                t = tag.pose_t.reshape(3, 1)
                x, y, z = t.flatten()

                # Only correct reported position, not the projection pose.
                if APPLY_OFFSET:
                    z -= DISTANCE_OFFSET
                distance = np.linalg.norm([x, y, z])
                roll, pitch, yaw = rotation_to_euler(R)

                rvec, _ = cv2.Rodrigues(R)
                cv2.drawFrameAxes(
                    display, camera_matrix, zero_distortion,
                    rvec, t, AXIS_LENGTH, 2
                )
                corners = tag.corners.astype(np.int32)
                cv2.polylines(display, [corners], True, (0, 255, 0), 2)
                center = tag.center.astype(int)

                lines = [
                    f"ID: {tag.tag_id}",
                    f"X: {x * 100:.2f} cm",
                    f"Y: {y * 100:.2f} cm",
                    f"Z: {z * 100:.2f} cm",
                    f"Distance: {distance * 100:.2f} cm",
                    f"Roll: {roll:.1f} deg",
                    f"Pitch: {pitch:.1f} deg",
                    f"Yaw: {yaw:.1f} deg"
                ]
                text_x = max(0, min(center[0] + 15, WIDTH - 200))
                text_y = max(20, min(center[1] - 40, HEIGHT - 155))
                for i, line in enumerate(lines):
                    cv2.putText(
                        display, line, (text_x, text_y + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
                    )
                print(" | ".join(lines))

            cv2.imshow("AprilTag Pose Detection", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if started:
            pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
