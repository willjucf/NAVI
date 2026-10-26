import cv2
import numpy as np
import ArducamDepthCamera as ac
import time

TOF_INDEX = 2
MAX_DEPTH_MM = 4000

print("Creating ToF camera...")

cam = ac.ArducamCamera()

print("Opening ToF on /dev/video%d..." % TOF_INDEX)
ret = cam.open(ac.Connection.CSI, TOF_INDEX)

if ret != 0:
    print("ERROR: open() failed:", ret)
    raise SystemExit

print("Starting depth stream...")
ret = cam.start(ac.FrameType.DEPTH)

if ret != 0:
    print("ERROR: start() failed:", ret)
    cam.close()
    raise SystemExit

print("Camera started.")
print("Press q in the window to quit.")

try:
    cam.setControl(ac.Control.RANGE, MAX_DEPTH_MM)
except Exception:
    pass

# Create a window immediately, even before a real frame arrives.
blank = np.zeros((480, 640, 3), dtype=np.uint8)

no_frame_count = 0

try:
    while True:

        frame = cam.requestFrame(200)

        if frame is None:
            no_frame_count += 1

            screen = blank.copy()

            cv2.putText(
                screen,
                "Waiting for ToF frame...",
                (100, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2
            )

            cv2.putText(
                screen,
                "No frames: %d" % no_frame_count,
                (100, 270),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            cv2.imshow("NAVI ToF Viewer", screen)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            continue

        try:
            depth = np.asarray(frame.depth_data, dtype=np.float32).copy()
        finally:
            cam.releaseFrame(frame)

        print(
            "frame:",
            depth.shape,
            "min:",
            np.min(depth),
            "max:",
            np.max(depth)
        )

        # Invalid pixels become 0
        valid = (depth > 0) & (depth <= MAX_DEPTH_MM)

        display = np.zeros(depth.shape, dtype=np.uint8)

        if np.any(valid):
            clipped = np.clip(depth, 0, MAX_DEPTH_MM)

            # Near = brighter, far = darker
            display[valid] = (
                255 - (clipped[valid] / MAX_DEPTH_MM * 255)
            ).astype(np.uint8)

        display = cv2.applyColorMap(display, cv2.COLORMAP_JET)
        display = cv2.resize(display, (640, 480))

        cv2.imshow("NAVI ToF Viewer", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    print("Closing camera...")
    try:
        cam.stop()
    except Exception:
        pass

    try:
        cam.close()
    except Exception:
        pass

    cv2.destroyAllWindows()
