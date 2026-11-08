# libs/YbProtocol.py

class YbProtocol:
    """模拟亚博原厂色块数据打包协议，保证打靶模式正常运行"""
    def __init__(self):
        pass

    def get_color_data(self, x, y, w, h):
        # 模拟返回打包数据，方便在终端以清晰易读的字典形式打印出来
        return {
            "x": int(x),
            "y": int(y),
            "width": int(w),
            "height": int(h)
        }