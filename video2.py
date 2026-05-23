import gc
import json
import socket
import time

import network
import nncase_runtime as nn
from media.sensor import *
from media.display import *
from media.media import *


WIFI_SSID = "嵌入式实验室406"
WIFI_PASSWORD = "qrssys406"
MODEL_PATH = "/sdcard/examples/kmodel/best.kmodel"
CLASSES = ["obj"]
HTTP_PORT = 8080
W, H = 320, 240
SHOW_LCD = False


def is_eagain(err):
    try:
        if getattr(err, "errno", None) == 11:
            return True
        args = getattr(err, "args", ())
        return bool(args and args[0] == 11) or "EAGAIN" in str(err)
    except Exception:
        return False


def safe_send(sock, data):
    for _ in range(20):
        try:
            sock.send(data)
            return True
        except Exception as err:
            if not is_eagain(err):
                raise
            time.sleep_ms(5)
    return False


def safe_accept(server):
    while True:
        try:
            return server.accept()
        except Exception as err:
            if not is_eagain(err):
                raise
            time.sleep_ms(20)


def wifi_ip():
    wlan = network.WLAN(network.STA_IF)
    print("WLAN 状态:", wlan.isconnected())
    
    if not wlan.isconnected():
        print("开始连接 WiFi:", WIFI_SSID)
        try:
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            for i in range(20):
                if wlan.isconnected():
                    print("WiFi 已连接")
                    break
                print("连接中...", i)
                time.sleep(1)
        except Exception as e:
            print("连接异常:", e)
    
    try:
        ip = wlan.ifconfig()[0]
        print("IP:", ip)
        return ip
    except:
        print("获取 IP 失败")
        return "0.0.0.0"


def init_hw():
    sensor = Sensor(width=W, height=H)
    sensor.reset()
    sensor.set_hmirror(True)
    sensor.set_vflip(True)
    sensor.set_framesize(width=W, height=H)
    sensor.set_pixformat(Sensor.RGB565)

    if SHOW_LCD:
        Display.init(Display.ST7701, width=W, height=H)

    MediaManager.init()
    sensor.run()

    kpu = nn.kpu()
    try:
        kpu.load_kmodel(MODEL_PATH)
    except Exception as err:
        print("加载模型失败:", err)
    return sensor, kpu


def html(ip):
    return (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n\r\n"
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>K230 YOLO</title>"
        "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
        ".wrap{position:relative;display:inline-block;max-width:100vw}"
        "img{display:block;max-width:100vw;max-height:100vh}"
        "canvas{position:absolute;left:0;top:0;pointer-events:none}</style>"
        "</head><body><div style='padding:8px'>K230 IP: " + ip + "</div>"
        "<div class='wrap'>"
        "<img id='view' src='/stream'>"
        "<canvas id='overlay'></canvas>"
        "</div>"
        "<script>"
        "const img=document.getElementById('view');"
        "const canvas=document.getElementById('overlay');"
        "const ctx=canvas.getContext('2d');"
        "const render=()=>{canvas.width=img.clientWidth;canvas.height=img.clientHeight;};"
        "img.onload=render;window.onresize=render;render();"
        "async function tick(){"
        "try{const r=await fetch('/boxes?t='+Date.now());const boxes=await r.json();"
        "render();ctx.clearRect(0,0,canvas.width,canvas.height);"
        "ctx.strokeStyle='red';ctx.lineWidth=2;"
        "const sx=canvas.width/320, sy=canvas.height/240;"
        "boxes.forEach(b=>{ctx.strokeRect(b.x*sx,b.y*sy,b.w*sx,b.h*sy);});}catch(e){}"
        "requestAnimationFrame(tick);}tick();"
        "</script></body></html>"
    )


def stream_header():
    return (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
    )


def json_header():
    return (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Cache-Control: no-cache\r\n\r\n"
    )


def pack_boxes(objs):
    boxes = []
    if objs:
        for obj in objs:
            if hasattr(obj, "rect"):
                x, y, w, h = obj.rect()
                cls = obj.classid() if hasattr(obj, "classid") else 0
                score = obj.value() if hasattr(obj, "value") else 0
            else:
                x, y, w, h = obj[0], obj[1], obj[2], obj[3]
                cls = obj[4] if len(obj) > 4 else 0
                score = obj[5] if len(obj) > 5 else 0
            boxes.append({"x": int(x), "y": int(y), "w": int(w), "h": int(h), "cls": int(cls), "score": float(score)})
    return boxes


def main():
    ip = wifi_ip()
    print("K230 IP:", ip)
    sensor, kpu = init_hw()

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
    last_boxes = []

    try:
        while True:
            cl, _ = safe_accept(server)
            print("客户端连接")

            try:
                try:
                    cl.settimeout(1)
                except Exception:
                    pass

                request = b""
                for _ in range(10):
                    try:
                        request = cl.recv(1024)
                        if request:
                            break
                    except Exception as err:
                        if not is_eagain(err):
                            raise
                    time.sleep_ms(50)

                first_line = request.split(b"\r\n", 1)[0] if request else b""
                print("请求:", first_line)

                if b"GET /boxes" in first_line:
                    print("返回坐标 JSON")
                    safe_send(cl, json_header())
                    safe_send(cl, json.dumps(last_boxes))
                    cl.close()
                    continue

                if b"GET /stream" not in first_line:
                    print("返回 HTML 页面")
                    safe_send(cl, html(ip))
                    cl.close()
                    continue

                print("开始推理循环")
                safe_send(cl, stream_header())

                while True:
                    frame_start = time.ticks_ms()
                    print("读取摄像头")

                    img = sensor.snapshot()
                    print("摄像头快照成功")
                    
                    # 先跳过推理，直接发图像
                    last_boxes = []

                    if SHOW_LCD:
                        try:
                            Display.show_image(img)
                        except Exception:
                            pass

                    jpg = img.compress(quality=40)
                    if not safe_send(cl, "--frame\r\n"):
                        break
                    if not safe_send(cl, "Content-Type: image/jpeg\r\n\r\n"):
                        break
                    if not safe_send(cl, jpg):
                        break
                    if not safe_send(cl, "\r\n"):
                        break

                    frame_cost = time.ticks_diff(time.ticks_ms(), frame_start)
                    if frame_cost < 1000:
                        time.sleep_ms(1000 - frame_cost)

                    if frame_start % 10 == 0:
                        gc.collect()

            except Exception as err:
                print("断开:", err)
            finally:
                try:
                    cl.close()
                except Exception:
                    pass

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


if __name__ == "__main__":
    main()