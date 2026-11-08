import time
from media.sensor import *
from media.display import *
from media.media import *


W=800
H=480


# ===========================
# 阈值
# ===========================

# 黑色粗边
BLACK_TH=[
    (0,25,-10,10,-10,10)
]

# 浅红色
RED_TH=[
    (20,70,20,70,-10,60)
]


# ===========================
# 检测黑色靶框
# ===========================

def find_board(img):

    blobs=img.find_blobs(
        BLACK_TH,
        pixels_threshold=3000,
        area_threshold=3000,
        merge=True
    )


    if not blobs:
        return None


    target=None
    max_score=0


    for b in blobs:


        x,y,w,h=b.rect()


        area=b.pixels()


        # 尺寸过滤

        if w<120 or h<120:
            continue


        if w>750 or h>450:
            continue


        ratio=w/h


        # 靶板接近矩形

        if ratio<0.6 or ratio>1.7:
            continue



        score=w*h


        if score>max_score:

            max_score=score
            target=b



    return target



# ===========================
# ROI寻找红色靶心
# ===========================
def find_red_center(img,board):

    x,y,w,h=board.rect()

    blobs=img.find_blobs(
        RED_TH,
        pixels_threshold=30,
        area_threshold=30,
        merge=True,
        roi=(x,y,w,h)
    )


    if not blobs:
        return None


    sx=0
    sy=0
    total=0


    cx0=x+w//2
    cy0=y+h//2


    for b in blobs:

        p=b.pixels()

        bw=b.w()
        bh=b.h()


        # 去小噪声
        if p<30:
            continue


        # 红环不应该特别细长

        r=bw/bh

        if r<0.3 or r>3:
            continue


        # 距离靶板中心太远不要

        dx=b.cx()-cx0
        dy=b.cy()-cy0

        dis=dx*dx+dy*dy


        if dis>(w*w+h*h)//4:
            continue


        sx+=b.cx()*p
        sy+=b.cy()*p
        total+=p


    if total==0:
        return None


    return (
        int(sx/total),
        int(sy/total),
        total
    )



# ===========================
# 摄像头初始化
# ===========================

def init_camera():

    sensor=Sensor()


    sensor.reset()


    sensor.set_framesize(
        width=W,
        height=H
    )


    sensor.set_pixformat(
        Sensor.RGB565
    )


    Display.init(
        Display.ST7701,
        width=W,
        height=H,
        to_ide=True
    )


    MediaManager.init()


    sensor.run()


    time.sleep(1)


    return sensor



# ===========================
# 主循环
# ===========================

def main():

    print("Target Detect Start")


    cam=init_camera()


    while True:


        img=cam.snapshot()


        board=find_board(img)


        if board:


            # 显示靶框

            img.draw_rectangle(
                board.rect(),
                color=(0,255,0),
                thickness=3
            )


            center=find_red_center(
                img,
                board
            )


            if center:


                cx,cy,area=center


                img.draw_cross(
                    cx,
                    cy,
                    color=(255,0,0),
                    size=20,
                    thickness=3
                )


                img.draw_string_advanced(
                    20,
                    40,
                    30,
                    "CENTER:%d,%d"%(cx,cy),
                    color=(255,0,0)
                )


                img.draw_string_advanced(
                    20,
                    80,
                    25,
                    "RED:%d"%area,
                    color=(255,255,0)
                )


            else:


                img.draw_string_advanced(
                    20,
                    40,
                    30,
                    "NO RED",
                    color=(255,100,100)
                )


        else:


            img.draw_string_advanced(
                20,
                40,
                30,
                "NO TARGET",
                color=(255,100,100)
            )


        # 图像中心

        img.draw_cross(
            W//2,
            H//2,
            color=(0,255,0),
            size=25,
            thickness=3
        )


        Display.show_image(img)



        time.sleep_ms(30)



if __name__=="__main__":

    try:

        main()

    except KeyboardInterrupt:

        print("Stop")

    finally:

        print("Exit")
