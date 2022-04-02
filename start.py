#!/usr/bin/env python3

import serial
import math
import numpy as np
import cv2 as cv
from random import random

window_name = 'image'

imgsize = 512

centerX = imgsize / 2
centerY = imgsize / 2
size = imgsize / 2


count = 0
arraysize = 100
values = [0] * arraysize
distances = [0] * 360
min_reflectivity = 10

def draw_lidar():
    print ("Distances", distances)
    img = np.zeros((imgsize,imgsize,3), np.uint8)
    for angle in range(360):
        if distances[angle] > 0:
            x = int(math.sin(angle * math.pi * 2 / 360) * distances[angle] * 0.1 + centerX)
            y = int(math.cos(angle * math.pi * 2 / 360) * distances[angle] * 0.1 + centerY)
            cv.circle(img,(x,y),2, (255,255,255),-1)
    cv.imshow(window_name, img)
    cv.waitKey(10)


def get_int(lb, hb):
    return lb | (hb << 8)

def process_lidar_data():
    global distances, values
    angle = (values[1] - 0xA0) * 4
    speed = get_int (values[2],values[3])
    distance = [0] * 4
    reflectivity = [0] * 4
    distance[0]     = get_int (values[4],values[5])
    reflectivity[0] = get_int (values[6],values[7])

    distance[1]     = get_int (values[8],values[9])
    reflectivity[1] = get_int (values[10],values[11])

    distance[2]     = get_int (values[12],values[13])
    reflectivity[2] = get_int (values[14],values[15])

    distance[3]     = get_int (values[16],values[17])
    reflectivity[3] = get_int (values[18],values[19])

    checksum2 = 0
    for x in range(20):
        checksum2 = checksum2 + values[x]
  
    checksum1 = get_int (values[20],values[21])
    if checksum1 == checksum2 and angle < 360:
        # print("Data: ", angle, speed, distance, reflectivity, checksum1, checksum2)
        if angle == 0:
            draw_lidar()
            distances = [0] * 360
        for x in range(4):
            if reflectivity[x] > min_reflectivity:
                distances[angle+x] = distance[x]
            else:
                distances[angle+x] = -1
    else:
        print ("Invalid data", checksum1, checksum2, angle)

# Connect to LDS-006 RX/TX
ser = serial.Serial('/dev/cu.usbserial-A50285BI', 115200, timeout=5)

# Start lidar sensor
ser.write(b'$');
ser.write(b"startlds$");

# Run lidar loop
while True:
    b = ser.read()
    val = int.from_bytes(b,byteorder='big')
    if val == 0xFA and count > 21:
        if count == 22:
            # Assume this is a valid data frame
            process_lidar_data()
        count = 0
        values = [0] * arraysize
        values[0] = val
    else:
        if count < arraysize:
            values[count] = val
    count = count + 1
ser.close()

cv.destroyAllWindows() 
