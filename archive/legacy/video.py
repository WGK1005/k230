import network
import time

# ====== WiFi信息 ======
SSID = "嵌入式实验室406"
PASSWORD = "qrssys406"

# ====== 创建WiFi对象 ======
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# ====== 连接WiFi ======
print("Connecting WiFi...")

wlan.connect(SSID, PASSWORD)

# ====== 等待连接成功 ======
timeout = 10  # 秒
start = time.time()

while not wlan.isconnected():
    if time.time() - start > timeout:
        print("WiFi connect timeout!")
        break
    time.sleep(1)

# ====== 结果输出 ======
if wlan.isconnected():
    print("WiFi Connected!")
    print("IP info:", wlan.ifconfig())
else:
    print("Failed to connect WiFi")
