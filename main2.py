# Description: This example demonstrates how to stream video and audio to the network using the RTSP server.
#
# Note: You will need an SD card to run this example.
#
# You can run the rtsp server to stream video and audio to the network

# ===== WiFi连接（新增） =====
import network
import time
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


# ============================
from media.vencoder import *
from media.sensor import *
from media.media import *
import time, os
import _thread
import multimedia as mm
# from time import *        ← 这行已删除

class RtspServer:
    def __init__(self,session_name="test",port=8554,video_type = mm.multi_media_type.media_h264,enable_audio=False):
        self.session_name = session_name # session name
        self.video_type = video_type  # 视频类型264/265
        self.enable_audio = enable_audio # 是否启用音频
        self.port = port   #rtsp 端口号
        self.rtspserver = mm.rtsp_server() # 实例化rtsp server
        self.venc_chn = VENC_CHN_ID_0 #venc通道
        self.start_stream = False #是否启动推流线程
        self.runthread_over = False #推流线程是否结束

    def start(self):
        self._init_stream()
        self.rtspserver.rtspserver_init(self.port)
        self.rtspserver.rtspserver_createsession(self.session_name,self.video_type,self.enable_audio)
        self.rtspserver.rtspserver_start()
        self._start_stream()
        self.start_stream = True
        _thread.start_new_thread(self._do_rtsp_stream,())

    def stop(self):
        if (self.start_stream == False):
            return
        self.start_stream = False
        while not self.runthread_over:
            time.sleep(0.1)                              # ← 改
        self.runthread_over = False
        self._stop_stream()
        self.rtspserver.rtspserver_stop()
        self.rtspserver.rtspserver_deinit()

    def get_rtsp_url(self):
        return self.rtspserver.rtspserver_getrtspurl(self.session_name)

    def _init_stream(self):
        width = 640
        height = 480
        width = ALIGN_UP(width, 16)
        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(width = width, height = height, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)
        self.encoder = Encoder()
        self.encoder.SetOutBufs(self.venc_chn, 8, width, height)
        self.link = MediaManager.link(self.sensor.bind_info()['src'], (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn))
        MediaManager.init()
        chnAttr = ChnAttrStr(self.encoder.PAYLOAD_TYPE_H264, self.encoder.H264_PROFILE_MAIN, width, height)
        self.encoder.Create(self.venc_chn, chnAttr)

    def _start_stream(self):
        self.encoder.Start(self.venc_chn)
        self.sensor.run()

    def _stop_stream(self):
        self.sensor.stop()
        del self.link
        self.encoder.Stop(self.venc_chn)
        self.encoder.Destroy(self.venc_chn)
        MediaManager.deinit()

    def _do_rtsp_stream(self):
        try:
            streamData = StreamData()
            while self.start_stream:
                os.exitpoint()
                self.encoder.GetStream(self.venc_chn, streamData)
                for pack_idx in range(0, streamData.pack_cnt):
                    stream_data = bytes(uctypes.bytearray_at(streamData.data[pack_idx], streamData.data_size[pack_idx]))
                    self.rtspserver.rtspserver_sendvideodata(self.session_name,stream_data, streamData.data_size[pack_idx],1000)
                self.encoder.ReleaseStream(self.venc_chn, streamData)
        except BaseException as e:
            import sys
            sys.print_exception(e)
        finally:
            self.runthread_over = True
            self.stop()
        self.runthread_over = True

if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    rtspserver = RtspServer()
    rtspserver.start()
    print("rtsp server start:", rtspserver.get_rtsp_url())

    while True:
        time.sleep(1)
