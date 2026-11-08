import network
import time

ssid = "嵌入式实验室406"
password = "qrssys406"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

while not wlan.isconnected():
    print("连接中...")
    time.sleep(1)

print("IP:", wlan.ifconfig()[0])
