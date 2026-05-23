import network
import time

SSID = "实验"
PASSWORD = "15235105794"

print("[WIFI] 连接中...")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected():
        print("[WIFI] 等待连接...")
        time.sleep(1)
print("[WIFI] 连接成功, IP:", wlan.ifconfig()[0])
