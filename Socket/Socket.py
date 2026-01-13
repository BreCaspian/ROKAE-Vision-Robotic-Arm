# Socket.py  (server)
import os
import socket
import sys
import threading
import time

import cv2
import numpy as np

sys.path.append(r"D:\Desktop\RoboMaster\RoboticArm\ROKAE\Workspace\CameraControl")
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT)

from Workspace.CameraControl.HikCamera import HikCamera

HOST = "0.0.0.0"
PORT = 8081

camera = HikCamera()

_display_lock = threading.Lock()
_display_running = False

# Vision -> motion params (tune after calibration)
_kx_mm_per_px = 0.838159
_ky_mm_per_px = 1.346629
_sign_x = 1.0
_sign_y = 1.0
_max_mm = 20.0
_deadband_mm = 50.0
_alpha = 0.35
_min_radius_px = 80
_max_radius_px = 600
_circularity_min = 0.65
_debug_vision = True
_vis_max_w = 960
_vis_max_h = 720
_debug_save = True
_debug_dir = os.path.join(os.path.dirname(__file__), "debug")
_fill_min = 0.45
_invert_feedback = True

_last_dx = 0.0
_last_dy = 0.0
_last_u = None
_last_v = None
_last_r = None
_max_jump_px = 400
_max_radius_jump_px = 120
_enable_jump_reject = False
_center_u = None
_center_v = None
_auto_set_center = True
_cache_dx = 0.0
_cache_dy = 0.0
_cache_rz = 0.0
_cache_ok = False
_cache_ts = 0.0


def _display_worker():
    global _display_running
    try:
        camera.display_stream()
    finally:
        with _display_lock:
            _display_running = False


def start_display():
    global _display_running
    with _display_lock:
        if _display_running:
            return False
        _display_running = True
    threading.Thread(target=_display_worker, daemon=True).start()
    return True


def send_tokens(conn, *tokens: str):
    payload = ("\r".join(tokens) + "\r").encode("ascii", errors="ignore")
    print("send raw:", repr(payload))
    conn.sendall(payload)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _deadband(v, band):
    return 0.0 if abs(v) < band else v


def _show_window(name, img):
    h, w = img.shape[:2]
    scale = min(_vis_max_w / float(w), _vis_max_h / float(h), 1.0)
    if scale < 1.0:
        img = cv2.resize(img, (int(w * scale), int(h * scale)))
    cv2.imshow(name, img)


def _init_debug_window():
    blank = np.zeros((720, 960, 3), dtype=np.uint8)
    cv2.putText(
        blank,
        "WAITING FOR TRIG",
        (30, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.4,
        (0, 200, 200),
        3,
    )
    _show_window("vision_vis", blank)
    cv2.waitKey(1)


def _to_gray_u8(img):
    if img is None:
        return None

    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    if gray.dtype != np.uint8:
        g = gray.astype(np.float32)
        gmin, gmax = float(g.min()), float(g.max())
        if gmax - gmin < 1e-6:
            return np.zeros_like(gray, dtype=np.uint8)
        g = (g - gmin) * (255.0 / (gmax - gmin))
        gray = g.astype(np.uint8)

    return gray


def _ensure_debug_dir():
    if _debug_save and not os.path.isdir(_debug_dir):
        os.makedirs(_debug_dir, exist_ok=True)


def _save_debug(name, img):
    if not _debug_save or img is None:
        return
    _ensure_debug_dir()
    path = os.path.join(_debug_dir, name)
    cv2.imwrite(path, img)


def _debug_show(gray, blur, norm, eq, th, vis, title="debug"):
    def resize(img, ww=520):
        if img is None:
            return None
        h, w = img.shape[:2]
        s = ww / float(w)
        return cv2.resize(img, (int(w * s), int(h * s)))

    _show_window(title + "_gray", resize(gray))
    _show_window(title + "_norm", resize(norm))
    _show_window(title + "_eq", resize(eq))
    _show_window(title + "_th", resize(th))
    if vis is not None:
        _show_window(title + "_vis", resize(vis))


def _find_circle_contours(gray_u8, invert=True):
    dbg = {}

    blur = cv2.GaussianBlur(gray_u8, (7, 7), 1.5)
    dbg["blur"] = blur

    bg = cv2.GaussianBlur(blur, (0, 0), 25)
    norm = cv2.divide(blur, bg, scale=255)
    dbg["norm"] = norm

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    eq = clahe.apply(norm)
    dbg["clahe"] = eq

    thresh_type = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    _ret, th = cv2.threshold(eq, 0, 255, thresh_type + cv2.THRESH_OTSU)
    dbg["th0"] = th

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=2)
    dbg["th"] = th

    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, dbg

    h, w = gray_u8.shape[:2]
    img_area = float(h * w)
    best = None
    best_score = -1.0
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < img_area * 0.0002 or area > img_area * 0.35:
            continue
        peri = float(cv2.arcLength(cnt, True))
        if peri < 1e-6:
            continue
        circularity = 4.0 * np.pi * area / (peri * peri)
        if circularity < _circularity_min:
            continue
        (u, v), r = cv2.minEnclosingCircle(cnt)
        if r < _min_radius_px or r > _max_radius_px:
            continue
        fill = area / (np.pi * r * r + 1e-6)
        if fill < _fill_min:
            continue
        score = (circularity * 10.0) + (fill * 3.0) + (r * 0.01)
        if score > best_score:
            best_score = score
            best = (float(u), float(v), float(r), float(circularity), area, float(fill), best_score)

    return best, dbg


def _find_circle_hough(gray):
    gray_blur = cv2.GaussianBlur(gray, (9, 9), 1.5)
    circles = cv2.HoughCircles(
        gray_blur,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=gray.shape[0] * 0.3,
        param1=120,
        param2=35,
        minRadius=_min_radius_px,
        maxRadius=_max_radius_px,
    )
    if circles is None:
        return None
    circles = np.squeeze(circles, axis=0)
    h, w = gray.shape[:2]
    cx, cy = w * 0.5, h * 0.5
    d2 = (circles[:, 0] - cx) ** 2 + (circles[:, 1] - cy) ** 2
    best = circles[int(np.argmin(d2))]
    u, v, r = float(best[0]), float(best[1]), float(best[2])
    return u, v, r


def detect_circle_offset_mm(img_bgr_or_mono):
    global _last_dx, _last_dy, _last_u, _last_v, _last_r, _center_u, _center_v

    if img_bgr_or_mono is None:
        if _debug_vision:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                "NO FRAME",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.4,
                (0, 0, 200),
                3,
            )
            _show_window("vision_vis", blank)
            cv2.waitKey(1)
            _save_debug("vision_vis.png", blank)
        return 0.0, 0.0, False

    gray = _to_gray_u8(img_bgr_or_mono)
    if gray is None:
        return 0.0, 0.0, False

    h, w = gray.shape[:2]
    if _center_u is None or _center_v is None:
        cx, cy = w * 0.5, h * 0.5
    else:
        cx, cy = _center_u, _center_v

    found1, dbg1 = _find_circle_contours(gray, invert=True)
    found2, dbg2 = _find_circle_contours(gray, invert=False)
    found = None
    dbg = None
    if found1 and found2:
        found = found1 if found1[-1] >= found2[-1] else found2
        dbg = dbg1 if found is found1 else dbg2
    elif found1:
        found, dbg = found1, dbg1
    elif found2:
        found, dbg = found2, dbg2
    else:
        hough = _find_circle_hough(gray)
        if hough is not None:
            u, v, r = hough
            found = (u, v, r, 1.0, np.pi * r * r, 1.0, 0.0)
            dbg = {"th": np.zeros_like(gray)}

    if found is None:
        if _debug_vision:
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.circle(vis, (int(cx), int(cy)), 6, (0, 255, 255), 2)
            cv2.putText(
                vis,
                "NO TARGET",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.4,
                (0, 0, 200),
                3,
            )
            _show_window("vision_vis", vis)
            cv2.waitKey(1)
            _save_debug("vision_vis.png", vis)
        return 0.0, 0.0, False
    u, v, _r, circ, area, fill, score = found
    if _auto_set_center and _center_u is None and _center_v is None:
        _center_u, _center_v = u, v

    if _enable_jump_reject and _last_u is not None and _last_v is not None:
        jump = ((u - _last_u) ** 2 + (v - _last_v) ** 2) ** 0.5
        if jump > _max_jump_px or (_last_r is not None and abs(_r - _last_r) > _max_radius_jump_px):
            if _debug_vision:
                vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv2.putText(
                    vis,
                    "REJECT JUMP",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.4,
                    (0, 0, 200),
                    3,
                )
                _show_window("vision_vis", vis)
                cv2.waitKey(1)
            return 0.0, 0.0, False

    diff_u = u - cx
    diff_v = v - cy

    dx = _sign_x * diff_v * _kx_mm_per_px
    dy = _sign_y * diff_u * _ky_mm_per_px

    dx = _deadband(dx, _deadband_mm)
    dy = _deadband(dy, _deadband_mm)
    dx = _clamp(dx, -_max_mm, _max_mm)
    dy = _clamp(dy, -_max_mm, _max_mm)

    dx = (1 - _alpha) * _last_dx + _alpha * dx
    dy = (1 - _alpha) * _last_dy + _alpha * dy
    if _invert_feedback:
        dx = -dx
        dy = -dy
    _last_dx, _last_dy = dx, dy

    _last_u, _last_v, _last_r = u, v, _r
    print(
        "[VISION] circle(u,v,r)=(%.1f,%.1f,%.1f) circ=%.3f area=%.0f fill=%.2f dx,dy(mm)=(%.2f,%.2f)"
        % (u, v, _r, circ, area, fill, dx, dy)
    )
    if _debug_vision:
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.line(vis, (int(cx - 40), int(cy)), (int(cx + 40), int(cy)), (0, 255, 255), 3)
        cv2.line(vis, (int(cx), int(cy - 40)), (int(cx), int(cy + 40)), (0, 255, 255), 3)
        cv2.circle(vis, (int(u), int(v)), int(_r), (0, 255, 0), 2)
        cv2.circle(vis, (int(u), int(v)), 3, (0, 0, 255), -1)
        cv2.putText(
            vis,
            f"dx={dx:.2f} dy={dy:.2f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )
        _show_window("vision_vis", vis)
        cv2.waitKey(1)
        _save_debug("vision_vis.png", vis)
    return dx, dy, True


def on_TRIG():
    global _cache_dx, _cache_dy, _cache_rz, _cache_ok, _cache_ts
    t0 = time.time()
    img = camera.capture_once()
    img2 = camera.capture_once()
    img = img2 if img2 is not None else img
    if img is not None:
        print("frame:", img.dtype, img.shape, "min/max:", int(img.min()), int(img.max()))
    dx, dy, ok = detect_circle_offset_mm(img)
    if ok:
        _cache_dx, _cache_dy = dx, dy
        _cache_rz = 0.0
        _cache_ok = True
    _cache_ts = time.time()
    print("[TRIG] dt=%.3fs ok=%s" % (_cache_ts - t0, ok))


def on_SET(conn):
    global _center_u, _center_v
    img = camera.capture_once()
    if img is None:
        send_tokens(conn, "ERR")
        return
    gray = _to_gray_u8(img)
    if gray is None:
        send_tokens(conn, "ERR")
        return
    found, _dbg = _find_circle_contours(gray, invert=True)
    if found is None:
        found, _dbg = _find_circle_contours(gray, invert=False)
    if found is None:
        send_tokens(conn, "ERR")
        return
    u, v, _r, _circ, _area, _fill, _score = found
    _center_u, _center_v = u, v
    send_tokens(conn, "OK")


def on_DW(conn):
    dx = _cache_dx if _cache_ok else 0.0
    dy = _cache_dy if _cache_ok else 0.0
    rz = _cache_rz if _cache_ok else 0.0
    send_tokens(conn, "DW", f"{dx:.3f}", f"{dy:.3f}", f"{rz:.3f}")


server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print("waiting...")

try:
    if _debug_vision:
        cv2.namedWindow("vision_vis", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("vision_vis", _vis_max_w, _vis_max_h)
        _init_debug_window()
    while True:
        conn, addr = server.accept()
        conn.settimeout(1.0)
        print("connected from", addr)

        try:
            buf = b""
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                print("recv chunk:", repr(data))
                buf += data
                while True:
                    r = buf.find(b"\r")
                    n = buf.find(b"\n")
                    idxs = [i for i in (r, n) if i != -1]
                    if not idxs:
                        break

                    k = min(idxs)
                    raw = buf[:k]
                    buf = buf[k + 1 :]
                    if buf[:1] in (b"\r", b"\n"):
                        buf = buf[1:]

                    text = raw.decode(errors="ignore").strip()
                    print("recv raw:", repr(raw), "text:", repr(text))

                    if text == "DW":
                        img = camera.capture_once()
                        dx, dy, ok = detect_circle_offset_mm(img)
                        val_dx = dx if ok else 0.0
                        val_dy = dy if ok else 0.0
                        val_rz = 0.0
                        tag = "1" if ok else "0"
                        send_tokens(conn, tag, f"{val_dx:.3f}", f"{val_dy:.3f}", f"{val_rz:.3f}")

                    elif text == "SET":
                        on_SET(conn)

                    elif text == "SHOW":
                        start_display()
                        send_tokens(conn, "OK")

                    else:
                        send_tokens(conn, "OK")

        except socket.timeout:
            continue
        except Exception as e:
            print("error:", e)
        finally:
            conn.close()

finally:
    camera.close()
    server.close()
