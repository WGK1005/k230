import cv2, socket, struct, numpy as np, time
from ultralytics import YOLO

# ===== 配置 =====
PORT = 8888
MODEL_PATH = "D:\YOLO\bag.v1i.yolov8\yolov8n.pt"
THRESHOLD = 0.5
# =================

def recvall(sock, n):
    buf = b''
    while n:
        d = sock.recv(n)
        if not d: return None
        buf += d; n -= len(d)
    return buf

# 加载模型
model = YOLO(MODEL_PATH)

# 等待K230连接
srv = socket.socket()
srv.bind(('0.0.0.0', PORT))
srv.listen(1)
print(f"等待K230连接... 端口:{PORT}")
conn, addr = srv.accept()
print(f"已连接: {addr}")

while True:
    # 接收图片
    size = struct.unpack('!I', recvall(conn, 4))[0]
    img_data = recvall(conn, size)
    if not img_data: break
    
    frame = cv2.imdecode(np.frombuffer(img_data, np.uint8), 1)
    if frame is None: continue

    # 识别
    results = model(frame, verbose=False)[0]
    for box in results.boxes:
        if float(box.conf[0]) < THRESHOLD: continue
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,0,255), 2)
        cv2.putText(frame, f"{results.names[int(box.cls[0])]} {box.conf[0]:.2f}", 
                    (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    cv2.imshow("Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

conn.close(); srv.close(); cv2.destroyAllWindows()