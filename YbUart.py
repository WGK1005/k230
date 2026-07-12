# ybUtils/YbUart.py
from machine import UART, FPIOA

class YbUart:
    def __init__(self, baudrate=115200):
        self._fpioa = FPIOA()
        # 庐山派用户串口2：TX:GPIO11, RX:GPIO12
        self._fpioa.set_function(11, FPIOA.UART2_TXD)
        self._fpioa.set_function(12, FPIOA.UART2_RXD)
        self._uart = UART(UART.UART2, baudrate=baudrate, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

    def write(self, buf):
        return self._uart.write(buf)

    def read(self, num=None):
        if num is None:
            return self._uart.read()
        return self._uart.read(num)

    def any(self):
        return self._uart.any()