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
#sensor.skip_frames(time = 20)       # 跳过帧

serial = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)
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

# 变量
laser = (100, 63)   # 激光坐标
road = []           # 基于当前 的 最优化路径
roadPass = 0        # 已经过的路口数
roadFlag = [0, 0]        # 下个路口下降上升的状态 + 升 - 降

def serial_write(msg):
    #time.sleep_ms(200)
    print(msg)
    serial.write(msg)

def serial_read():
    while 1:
        msg = serial.read()
        if msg:
            print("----------")
            print(msg)
            print("----------")
            return msg

def findRect():
    while 1:
        clock.tick()
        img = sensor.snapshot()
        for r in img.find_rects(threshold = 20000):
            if r.w() > 20 and r.h() > 20:

                img.draw_rectangle(r.rect(), color = (255, 0, 0), scale = 4)
                corner = r.corners()
                img.draw_circle(corner[0][0], corner[0][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
                img.draw_circle(corner[1][0], corner[1][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
                img.draw_circle(corner[2][0], corner[2][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
                img.draw_circle(corner[3][0], corner[3][1], 5, color = (0, 0, 255), thickness = 2, fill = False)

                rec = []
                for i in range(0, 4):
                    rec.append([0, 0])

                if r.x() > 0 or r.y()> 0:
                    maxN = -10000
                    minN = 99999
                    for i in range(0, 4):
                        if(maxN < corner[i][0]):
                            maxN = corner[i][0]
                            rec[2] = corner[i]
                        if(minN > corner[i][0]):
                            minN = corner[i][0]
                            rec[0] = corner[i]

                    for i in range(0, 4):
                        maxN = -10000
                        minN = 99999
                        if(maxN < corner[i][1]):
                            maxN = corner[i][1]
                            rec[3] = corner[i]
                        if(minN > corner[i][1]):
                            minN = corner[i][1]
                            rec[1] = corner[i]

                    if(rec[0] == rec[1] or rec[0] == rec[3]):
                        rec[0] = [r.x(), r.y()]
                        rec[1] = [r.x() + r.w(), r.y()]
                        rec[2] = [r.x() + r.w(), r.y() + r.h()]
                        rec[3] = [r.x(), r.y() + r.h()]

                    dm = 99999
                    tar = 0
                    road = []

                    print("=======debug3========")
                    print(int(math.pow(rec[i][0]-laser[0], 2)))
                    for i in range(0, 4):
                        d = math.sqrt(int(math.pow(rec[i][0]-laser[0], 2)) + int(math.pow(rec[i][1]-laser[1], 2)))
                        d = int(d)
                        if dm > d:
                            tar = i
                    i = tar
                    while i <= 3:
                        road.append(rec[i])
                        i += 1

                    i = -1
                    while i != tar:
                        i += 1
                        road.append(rec[i])

                    print("=======debug4========")
                    print(road)
                    return road

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

def nextPos(road, begin = 0):
    rec = findRect()

    if begin:
        x = road[0][0]
        y = road[0][1]
    else:
        if road[0][0] - laser[0] < roadFlag[0]:
            x = road[0][0]
            y = road[0][1]
        elif road[0][0] - laser[0] > roadFlag[0]:
            x = road[1][0]
            y = road[1][1]
        else:
            pass

        if road[0][1] - laser[1] < roadFlag[1]:
            x = road[0][0]
            y = road[0][1]
        elif road[0][1] - laser[1] > roadFlag[1]:
            x = road[1][0]
            y = road[1][1]
        else:
            pass


    while abs(laser[0] - x) > 2 and abs(laser[1] - y) > 2:
        rec = findRect()

        if begin:
            x = road[0][0]
            y = road[0][1]
        else:
            if road[0][0] - laser[0] < roadFlag[0]:
                x = road[0][0]
                y = road[0][1]
            elif road[0][0] - laser[0] > roadFlag[0]:
                x = road[1][0]
                y = road[1][1]
            else:
                pass

            if road[0][1] - laser[1] < roadFlag[1]:
                x = road[0][0]
                y = road[0][1]
            elif road[0][1] - laser[1] > roadFlag[1]:
                x = road[1][0]
                y = road[1][1]
            else:
                pass

        if laser[0] > x:
            serial_write('3')
        else:
            serial_write('4')
        if laser[1] > y:
            serial_write('1')
        else:
            serial_write('2')

        msg = serial_read()
        while msg != b'9':
            msg = serial_read()
            pass

    while abs(laser[0] - x) > 2:
        rec = findRect()

        if begin:
            x = road[0][0]
            y = road[0][1]
        else:
            if road[0][0] - laser[0] < roadFlag[0]:
                x = road[0][0]
                y = road[0][1]
            elif road[0][0] - laser[0] > roadFlag[0]:
                x = road[1][0]
                y = road[1][1]
            else:
                pass

            if road[0][1] - laser[1] < roadFlag[1]:
                x = road[0][0]
                y = road[0][1]
            elif road[0][1] - laser[1] > roadFlag[1]:
                x = road[1][0]
                y = road[1][1]
            else:
                pass

        if laser[0] > x:
            serial_write('3')
        else:
            serial_write('4')

        msg = serial_read()
        while msg != b'9':
            msg = serial_read()
            pass

    while abs(laser[1] - y) > 2:
        rec = findRect()

        if begin:
            x = road[0][0]
            y = road[0][1]
        else:
            if road[0][0] - laser[0] < roadFlag[0]:
                x = road[0][0]
                y = road[0][1]
            elif road[0][0] - laser[0] > roadFlag[0]:
                x = road[1][0]
                y = road[1][1]
            else:
                pass

            if road[0][1] - laser[1] < roadFlag[1]:
                x = road[0][0]
                y = road[0][1]
            elif road[0][1] - laser[1] > roadFlag[1]:
                x = road[1][0]
                y = road[1][1]
            else:
                pass

        if laser[1] > y:
            serial_write('1')
        else:
            serial_write('2')

        msg = serial_read()
        while msg != b'9':
            msg = serial_read()
            pass
    return findRect()


while(True):
    clock.tick()
    road = findRect()

    road = nextPos(road)
    roadFlag = [road[1][0] - road[0][0], road[1][1] - road[0][1]]
    road = nextPos(road)
    roadFlag = [road[1][0] - road[0][0], road[1][1] - road[0][1]]
    road = nextPos(road)
    roadFlag = [road[1][0] - road[0][0], road[1][1] - road[0][1]]
    road = nextPos(road)
    roadFlag = [road[1][0] - road[0][0], road[1][1] - road[0][1]]
    road = nextPos(road)

    #nextPos(0, 1)

    #if(flag):
    #print(tar)
    #serial.deinit()
    #del serial
