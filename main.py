import gc
import socket
import time

import network
from media.sensor import *
from media.display import *
from media.media import *


# ===================== 最简配置 =====================
WIFI_SSID = "嵌入式实验室406"
WIFI_PASSWORD = "qrssys406"

W, H = 320, 240
CX, CY = W // 2, H // 2

RED_TH = (20, 80, 30, 100, 0, 60)
MIN_PIXELS = 200
HTTP_PORT = 8080
SHOW_LCD = False


def ensure_wifi():
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)

    if wlan.isconnected():
        return wlan.ifconfig()[0]

    try:
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    except Exception as err:
        print("Wi-Fi 连接失败:", err)

    for _ in range(60):
        if wlan.isconnected():
            return wlan.ifconfig()[0]
        time.sleep(1)

    return "0.0.0.0"


def init_camera():
    sensor = Sensor(width=W, height=H)
    sensor.reset()
    sensor.set_hmirror(True)
    sensor.set_vflip(True)
    sensor.set_framesize(width=W, height=H)
    sensor.set_pixformat(Sensor.RGB565)

    if SHOW_LCD:
        Display.init(Display.ST7701, width=W, height=H)

    MediaManager.init()
    time.sleep_ms(300)

    last_err = None
    for _ in range(3):
        try:
            sensor.run()
            return sensor
        except Exception as err:
            last_err = err
            print("sensor.run 重试:", err)
            time.sleep_ms(300)

    raise last_err

    return sensor


def draw_ui(img, blob, fps, found):
    if found and blob is not None:
        x, y, w, h = blob.rect()
        img.draw_rectangle(x, y, w, h, color=(255, 0, 0), thickness=2)


def send_html(cl, ip):
    page = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n"
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>K230 Red Stream</title>"
        "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;display:flex;align-items:center;justify-content:center;height:100vh}"
        ".box{text-align:center}img{max-width:100vw;max-height:100vh;border:1px solid #333;background:#000}</style>"
        "</head><body><div class='box'>"
        "<div>浏览器打开：</div>"
        f"<div>http://{ip}:{HTTP_PORT}/stream</div>"
        "<img src='/stream' alt='stream'>"
        "</div></body></html>"
    )
    cl.send(page)


def send_stream_header(cl):
    cl.send(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n"
    )


def main():
    print("=" * 40)
    print("K230 红色物体最简版本")
    print("=" * 40)

    ip = ensure_wifi()
    print("K230 IP:", ip)

    sensor = init_camera()

    addr = socket.getaddrinfo("0.0.0.0", HTTP_PORT)[0][-1]
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(addr)
    server.listen(1)
    try:
        server.settimeout(1)
    except Exception:
        pass

    print("浏览器打开: http://{}:{}/".format(ip, HTTP_PORT))
    print("或者直接打开: http://{}:{}/stream".format(ip, HTTP_PORT))

    frame_count = 0
    last_fps_time = time.ticks_ms()
    fps = 0

    try:
        while True:
            try:
                cl, client_addr = server.accept()
            except Exception:
                time.sleep_ms(10)
                continue

            print("客户端连接:", client_addr)

            try:
                try:
                    cl.settimeout(1)
                except Exception:
                    pass

                request = b""
                for _ in range(20):
                    try:
                        request = cl.recv(1024)
                        if request:
                            break
                    except Exception:
                        pass
                    time.sleep_ms(10)

                first_line = request.split(b"\r\n", 1)[0] if request else b""

                if b"GET /stream" in first_line:
                    send_stream_header(cl)

                    while True:
                        frame_start = time.ticks_ms()

                        img = sensor.snapshot()
                        blobs = img.find_blobs([RED_TH], pixels_threshold=MIN_PIXELS, merge=True)

                        found = False
                        blob = None
                        if blobs:
                            blob = max(blobs, key=lambda x: x.pixels())
                            found = True

                        draw_ui(img, blob, fps, found)

                        if SHOW_LCD:
                            try:
                                Display.show_image(img)
                            except Exception:
                                pass

                        jpg = img.compress(quality=35)
                        cl.send("--frame\r\n")
                        cl.send("Content-Type: image/jpeg\r\n\r\n")
                        cl.send(jpg)
                        cl.send("\r\n")

                        frame_count += 1
                        if frame_count % 20 == 0:
                            now = time.ticks_ms()
                            elapsed = time.ticks_diff(now, last_fps_time)
                            fps = 20000 // max(1, elapsed)
                            last_fps_time = now

                        if frame_count % 10 == 0:
                            gc.collect()

                        frame_cost = time.ticks_diff(time.ticks_ms(), frame_start)
                        if frame_cost < 15:
                            time.sleep_ms(15 - frame_cost)

                else:
                    send_html(cl, ip)

            except Exception as e:
                print("客户端断开:", e)
            finally:
                try:
                    cl.close()
                except Exception:
                    pass

    except KeyboardInterrupt:
        print("用户中断")
    finally:
        try:
            server.close()
        except Exception:
            pass

        try:
            sensor.stop()
        except Exception:
            pass

        if SHOW_LCD:
            try:
                Display.deinit()
            except Exception:
                pass

        try:
            MediaManager.deinit()
        except Exception:
            pass

        print("停止")


if __name__ == "__main__":
    main()
