import time
import network

def ap_test():
    # 1. 实例化 AP 接口对象
    ap = network.WLAN(network.AP_IF)

    # 2. 💥 关键点：必须显式激活 AP 模式
    ap.active(False)

    # 3. 配置热点 SSID 和 密码 (密码至少需要 8 位)
    ap.config(ssid='k230_ap_wjx', key='12345678')

    # 4. 给予硬件 1~2 秒启动和配置的时间
    time.sleep(1)

    # 5. 查看 AP 信息与状态
    print("--------------------------------")
    print("热点已成功开启！")

    try:
        # 获取网卡 IP 配置信息 (IP, 子网掩码, 网关, DNS)
        print("IP 配置信息:", ap.ifconfig())
    except AttributeError:
        pass

    # 打印 status 和 info
    print("AP Status:", ap.status())

    # 注意：部分固件版本可能没有 info() 方法，若报错可捕获或去掉
    try:
        print("AP Info:", ap.info())
    except Exception as e:
        print("AP Info 未提供或格式不兼容:", e)

    print("--------------------------------")

# 执行测试
ap_test()
