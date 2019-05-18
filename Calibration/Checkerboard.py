#!/usr/bin/python
#-*- coding:utf-8 -*-

import cv2
import numpy as np

WIDTH = 1240
HEIGHT = 1754
RGB_BLACK = [0, 0, 0]
RGB_WHITE = [255, 255, 255]

row = []
col = []

for rows in range(0, WIDTH):
	for cols in range(0, HEIGHT):
		if (76 < cols < 1677) and (19 < rows < 1220):
			if (int((cols - 77) / 200) % 2 != 0) and (int((rows - 20) / 200) % 2 != 0):
				col.append(RGB_BLACK)
			elif (int((cols - 77) / 200) % 2 == 0) and (int((rows - 20) / 200) % 2 == 0):
				col.append(RGB_BLACK)
			else:
				col.append(RGB_WHITE)
		else:
			col.append(RGB_WHITE)
	row.append(col)
	col = []

img = np.array(row, dtype = np.uint8)
cv2.imwrite( "Checkerboard.png", img )
