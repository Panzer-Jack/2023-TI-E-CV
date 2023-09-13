import sensor, image, time, lcd, math
from fpioa_manager import fm
from machine import UART

fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # 设置图像色彩格式为RGB565格式
sensor.set_framesize(sensor.QQVGA)  # 设置图像大小为160*120
sensor.set_auto_whitebal(False)      # 设置自动白平衡
sensor.set_brightness(3000)         # 设置亮度为3000
sensor.skip_frames(time = 20)       # 跳过帧

serial = UART(UART.UART2, 9600, 8, timeout=1000, read_buf_len=4096)
clock = time.clock()
corner = 0

# 任务
# task3 -- 识别点坐标·控制激光移动
# task4 -- 识别点坐标·控制激光移动

# 阈值区
# 正确
#red_threshold = (213, 255) #max min
#green_threshold = (201, 255) #max min
# 白天
red_threshold = (185, 255) #max min
green_threshold = (165, 255) #max min


# 权重
W = 1.8
laser = (101, 64)

def serial_write(msg):
    time.sleep_ms(50)
    print(msg)
    serial.write(msg)
    msg = serial_read()
    while not msg:
        msg = serial_read()

def serial_read():
    msg = serial.read()
    if msg:
        print("----------")
        print(msg)
        print("----------")
    return msg

def findRect():
    while 1:
        rec = []
        for i in range(0, 4):
            rec.append([0, 0])
        clock.tick()
        img = sensor.snapshot()
        for r in img.find_rects(threshold = 10000):
            if r.w() > 20 and r.h() > 20:
                img.draw_rectangle(r.rect(), color = (255, 0, 0), scale = 4)
                corner = r.corners()
                img.draw_circle(corner[0][0], corner[0][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
                img.draw_circle(corner[1][0], corner[1][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
                img.draw_circle(corner[2][0], corner[2][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
                img.draw_circle(corner[3][0], corner[3][1], 5, color = (0, 0, 255), thickness = 2, fill = False)

                if r.x() > 0 or r.y()> 0:
                    rec[0] = [r.x() - 1, r.y() - 2]
                    rec[1] = [r.x() + r.w() - 1, r.y() - 2]
                    rec[2] = [r.x() + r.w() - 1, r.y() + r.h() - 2]
                    rec[3] = [r.x() - 1, r.y() + r.h() - 2]
                    print(rec)
                    return rec

def findLaser():
    while 1:
        clock.tick()
        img = sensor.snapshot()
        for blob in img.find_blobs([red_threshold],pixels_threshold=1, area_threshold=2):
            img.draw_cross(blob.cx(), blob.cy(), color = (0, 0, 255))
            laserX = blob.cx()
            laserY = blob.cy()
            if laserX > 0 and laserY > 0:
                laser = (laserX, laserY)
                print(laser)
                return laser



def nextLaser(tarX, tarY):
    i = 0
    for i in range(0, min(abs(tarX), abs(tarY))):
        if tarX > 0 and tarY > 0:
            serial_write('3')
            serial_write('1')
        elif tarX < 0 and tarY < 0:
            serial_write('4')
            serial_write('2')
        elif tarX > 0 and tarY < 0:
            serial_write('3')
            serial_write('2')
        else:
            serial_write('4')
            serial_write('1')
    while i < abs(tarX):
        i += 1
        if tarX < 0:
            serial_write('4')
        else:
            serial_write('3')
    while i < abs(tarY):
        i += 1
        if tarY < 0:
            serial_write('2')
        else:
            serial_write('1')

    rec = findRect()
    dm = 9999
    tar = []
    for i in range(0, 4):
        d = math.sqrt(abs(math.pow(rec[i][0]-laser[0], 2))+abs(math.pow(rec[i][1]-laser[1], 2)))
        if dm > d:
            dm = d
            tar = rec[i]
            print("tar: ")
            print(tar)
    while abs(laser[0] - tar[0]) > 2:
        rec = findRect()
        dm = 9999
        tar = []
        for i in range(0, 4):
            d = math.sqrt(abs(math.pow(rec[i][0]-laser[0], 2))+abs(math.pow(rec[i][1]-laser[1], 2)))
            if dm > d:
                dm = d
                tar = rec[i]
                print("tar: ")
                print(tar)
        if laser[0] > tar[0]:
            serial_write('3')
        else:
            serial_write('4')

    while abs(laser[1] - tar[1]) > 2:
        rec = findRect()
        dm = 9999
        tar = []
        for i in range(0, 4):
            d = math.sqrt(abs(math.pow(rec[i][0]-laser[0], 2))+abs(math.pow(rec[i][1]-laser[1], 2)))
            if dm > d:
                dm = d
                tar = rec[i]

                print("tar: ")
                print(tar)
        if laser[1] > tar[1]:
            serial_write('1')
        else:
            serial_write('2')

rec = findRect()
while(True):
    clock.tick()

    rec = findRect()
    print(rec)
    tarX = int((laser[0] - rec[0][0]) * W)
    tarY = int((laser[1] - rec[0][1]) * W)
    print("===========")
    print(tarX, tarY)
    print("===========")
    nextLaser(tarX, tarY)

    rec = findRect()
    print(rec)
    tarX = int((laser[0] - rec[1][0]) * W)
    tarY = int((laser[1] - rec[1][1]) * W)
    print("===========")
    print(tarX, tarY)
    print("===========")
    nextLaser(tarX, tarY)

    rec = findRect()
    print(rec)
    tarX = int((laser[0] - rec[2][0]) * W)
    tarY = int((laser[1] - rec[2][1]) * W)
    print("===========")
    print(tarX, tarY)
    print("===========")
    nextLaser(tarX, tarY)


    rec = findRect()
    print(rec)
    tarX = int((laser[0] - rec[3][0]) * W)
    tarY = int((laser[1] - rec[3][1]) * W)
    print("===========")
    print(tarX, tarY)
    print("===========")
    nextLaser(tarX, tarY)

    rec = findRect()
    print(rec)
    tarX = int((laser[0] - rec[0][0]) * W)
    tarY = int((laser[1] - rec[0][1]) * W)
    print("===========")
    print(tarX, tarY)
    print("===========")
    nextLaser(tarX, tarY)

    break

