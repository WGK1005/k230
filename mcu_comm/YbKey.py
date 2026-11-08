# ybUtils/YbKey.py
from machine import FPIOA, Pin

class YbKey:
    def __init__(self):
        self._fpioa = FPIOA()
        self._pin_num = 53  # 庐山派板载USR按键为 GPIO53
        self._fpioa.set_function(self._pin_num, FPIOA.GPIO53)
        # 庐山派按键电路无板载上拉，配置为输入，开启内部下拉
        self._key = Pin(self._pin_num, Pin.IN, Pin.PULL_DOWN)

    def value(self):
        return self._key.value()

    def is_pressed(self):
        # 庐山派按键按下时为高电平 (1)
        return self._key.value() == 1