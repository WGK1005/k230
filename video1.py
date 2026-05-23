import network, socket, time
import image, lcd, camera, nncase_runtime as nn

# ===== WiFi（你已经连了，这里只是获取IP）=====
ip = network.WLAN(network.STA_IF).ifconfig()[0]
print("K230 IP:", ip)

# ===== LCD =====
lcd.init()

# ===== 摄像头 =====
camera.init()
camera.set_framesize(320, 240)

# ===== YOLO模型加载 =====
model_path = "/sdcard/model.kmodel"   # ⚠️改成你的路径
kpu = nn.kpu()
kpu.load_kmodel(model_path)

# ===== 类别（自己改）=====
classes = ["leaf"]

# ===== HTTP服务器 =====
addr = socket.getaddrinfo('0.0.0.0', 8080)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)

print("浏览器打开:", "http://"+ip+":8080")

# ===== 主循环 =====
while True:
    cl, addr = s.accept()
    print("客户端连接")

    cl.send("HTTP/1.1 200 OK\r\n")
    cl.send("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n")

    try:
        while True:
            # ===== 获取图像 =====
            img = camera.capture()

            # ===== YOLO推理 =====
            objs = kpu.run_yolo2(img, 0.5, 0.3)

            # ===== 画框 =====
            if objs:
                for obj in objs:
                    x, y, w, h = obj.rect()
                    img.draw_rectangle(x, y, w, h, color=(255,0,0))
                    img.draw_string(x, y, classes[obj.classid()], scale=2)

            # ===== 显示到屏幕 =====
            lcd.display(img)

            # ===== 转JPEG =====
            jpg = img.compress(quality=50)

            # ===== 推流 =====
            cl.send("--frame\r\n")
            cl.send("Content-Type: image/jpeg\r\n\r\n")
            cl.send(jpg)
            cl.send("\r\n")

    except Exception as e:
        print("断开:", e)
        cl.close()