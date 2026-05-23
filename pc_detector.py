"""
电脑端 —— VLC中转拉流 + YOLOv8多线程检测
"""
import cv2
from ultralytics import YOLO
import threading

# ===== 配置 =====
STREAM_URL = "http://127.0.0.1:8080/video"  # VLC中转地址
MODEL_PATH = r"D:\YOLO\best.pt"
THRESHOLD = 0.5
RESIZE = (320, 320)
# =================

print("[YOLO] 加载模型...")
model = YOLO(MODEL_PATH)
print("[YOLO] 完成")

cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print("连接失败！")
    exit()

# 共享变量
current_frame = None
current_results = None
lock = threading.Lock()
running = True

def capture_thread():
    """拉流线程"""
    global current_frame, running
    while running:
        ret, frame = cap.read()
        if ret:
            with lock:
                current_frame = frame.copy()

def detect_thread():
    """YOLO识别线程"""
    global current_frame, current_results, running
    while running:
        with lock:
            if current_frame is None:
                continue
            small = cv2.resize(current_frame, RESIZE)
        results = model(small, verbose=False)[0]
        with lock:
            current_results = results

threading.Thread(target=capture_thread, daemon=True).start()
threading.Thread(target=detect_thread, daemon=True).start()

print("[SYS] 开始检测，按Q退出")

while True:
    with lock:
        if current_frame is None:
            continue
        display = current_frame.copy()
        results = current_results

    if results:
        h_ratio = display.shape[0] / RESIZE[1]
        w_ratio = display.shape[1] / RESIZE[0]
        for box in results.boxes:
            if float(box.conf[0]) < THRESHOLD:
                continue
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1 = int(x1*w_ratio), int(y1*h_ratio)
            x2, y2 = int(x2*w_ratio), int(y2*h_ratio)
            cv2.rectangle(display, (x1,y1), (x2,y2), (0,0,255), 2)
            cv2.putText(display, f"{results.names[int(box.cls[0])]} {box.conf[0]:.2f}",
                        (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    cv2.imshow("Landslide Detection", display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

running = False
cap.release()
cv2.destroyAllWindows()