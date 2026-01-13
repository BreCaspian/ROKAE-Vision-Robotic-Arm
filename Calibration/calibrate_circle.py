# calibrate_circle.py
import os
import sys
import cv2
import numpy as np
import math

sys.path.append(r"D:\Desktop\RoboMaster\RoboticArm\ROKAE\Workspace\CameraControl")
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT)

from Workspace.CameraControl.HikCamera import HikCamera


def _debug_show(gray, blur, norm, eq, th, vis, title="debug"):
    def resize(img, ww=520):
        if img is None:
            return None
        h, w = img.shape[:2]
        s = ww / float(w)
        return cv2.resize(img, (int(w * s), int(h * s)))

    cv2.imshow(title + "_gray", resize(gray))
    cv2.imshow(title + "_norm", resize(norm))
    cv2.imshow(title + "_eq", resize(eq))
    cv2.imshow(title + "_th", resize(th))
    if vis is not None:
        cv2.imshow(title + "_vis", resize(vis))


def detect_circle_center(img_bgr_or_mono, debug=False):
    if img_bgr_or_mono is None:
        return None

    if len(img_bgr_or_mono.shape) == 3:
        gray = cv2.cvtColor(img_bgr_or_mono, cv2.COLOR_BGR2GRAY)
    else:
        gray = img_bgr_or_mono

    h, w = gray.shape[:2]
    blur = cv2.GaussianBlur(gray, (7, 7), 1.5)
    bg = cv2.GaussianBlur(blur, (0, 0), 25)
    norm = cv2.divide(blur, bg, scale=255)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    eq = clahe.apply(norm)

    _ret, th = cv2.threshold(eq, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, k, iterations=1)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        if debug:
            _debug_show(gray, blur, norm, eq, th, None, title="debug")
        return None

    img_area = h * w
    best = None
    best_score = -1.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < img_area * 0.0005 or area > img_area * 0.2:
            continue
        peri = cv2.arcLength(c, True)
        if peri < 1e-6:
            continue
        circularity = 4 * math.pi * area / (peri * peri)
        if circularity < 0.72:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        if r < 20 or r > min(h, w) * 0.35:
            continue
        score = circularity * 10.0 + r * 0.01
        if score > best_score:
            best_score = score
            best = (x, y, r, circularity)

    if best is None:
        if debug:
            _debug_show(gray, blur, norm, eq, th, None, title="debug_no_best")
        return None

    x, y, r, circ = best
    if debug:
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.circle(vis, (int(x), int(y)), int(r), (0, 255, 0), 2)
        cv2.circle(vis, (int(x), int(y)), 3, (0, 0, 255), -1)
        cv2.putText(
            vis,
            f"r={r:.1f}, circ={circ:.3f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        _debug_show(gray, blur, norm, eq, th, vis, title="debug_ok")

    return float(x), float(y)


def capture_center(camera):
    img = camera.grab_frame()
    center = detect_circle_center(img, debug=True)
    return center, img


def _show_preview(img, center, title, max_w=960, max_h=720):
    if img is None:
        vis = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            vis,
            "No frame",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
        )
    elif len(img.shape) == 2:
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        vis = img.copy()
    h, w = vis.shape[:2]
    cv2.circle(vis, (int(w // 2), int(h // 2)), 6, (0, 255, 255), 2)
    if center is not None:
        u, v = int(center[0]), int(center[1])
        cv2.circle(vis, (u, v), 8, (0, 255, 0), 2)
        cv2.putText(
            vis,
            f"OK ({u},{v})",
            (u + 10, v - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 180, 0),
            3,
        )
    else:
        cv2.putText(
            vis,
            "NO TARGET",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.4,
            (0, 0, 200),
            3,
        )
    h, w = vis.shape[:2]
    scale = min(max_w / float(w), max_h / float(h), 1.0)
    if scale < 1.0:
        vis = cv2.resize(vis, (int(w * scale), int(h * scale)))
    cv2.imshow(title, vis)


def wait_for_capture(camera, title):
    print("Press 'c' to capture, 'q' to quit. Click the window to focus.")
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 720)
    while True:
        center, img = capture_center(camera)
        _show_preview(img, center, title)
        if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            raise KeyboardInterrupt
        if key == ord("c"):
            if center is None:
                print("No circle found, try again...")
                continue
            cv2.destroyWindow(title)
            return center


def prompt_axis(axis_name):
    print(f"\n=== Calibrate {axis_name}-axis ===")
    print("Place the circle near image center.")
    u1, v1 = wait_for_capture(camera, f"Capture A ({axis_name})")
    print(f"A: (u, v)=({u1:.1f}, {v1:.1f})")

    print(f"Move target along robot +{axis_name} by a known distance (mm).")
    u2, v2 = wait_for_capture(camera, f"Capture B ({axis_name})")
    print(f"B: (u, v)=({u2:.1f}, {v2:.1f})")

    dist_mm = float(input(f"Enter the moved distance along +{axis_name} (mm): ").strip())
    du = u2 - u1
    dv = v2 - v1

    if axis_name.upper() == "X":
        sign = 1.0 if du >= 0 else -1.0
        k = abs(dist_mm / du) if du != 0 else 0.0
        print(f"du={du:.2f} px, kx={k:.6f} mm/px, sign_x={sign:+.0f}")
        return {"k": k, "sign": sign}
    else:
        sign = 1.0 if dv >= 0 else -1.0
        k = abs(dist_mm / dv) if dv != 0 else 0.0
        print(f"dv={dv:.2f} px, ky={k:.6f} mm/px, sign_y={sign:+.0f}")
        return {"k": k, "sign": sign}


if __name__ == "__main__":
    camera = HikCamera()
    try:
        camera.set_trigger_mode(False)
        print("Calibration tool started.")
        print("Make sure the circle is visible in the camera view.")

        do_x = input("Calibrate X axis? (y/n): ").strip().lower().startswith("y")
        do_y = input("Calibrate Y axis? (y/n): ").strip().lower().startswith("y")

        result_x = prompt_axis("X") if do_x else None
        result_y = prompt_axis("Y") if do_y else None

        print("\n=== Suggested Socket.py params ===")
        if result_x:
            print(f"_k_mm_per_px = {result_x['k']:.6f}")
            print(f"_sign_x = {result_x['sign']:+.1f}")
        if result_y:
            print(f"_sign_y = {result_y['sign']:+.1f}")
            if result_x is None:
                print(f"_k_mm_per_px = {result_y['k']:.6f}")
        if result_x and result_y:
            print("# If X/Y scale differs, consider separate kx/ky in code.")

    finally:
        cv2.destroyAllWindows()
        camera.close()
