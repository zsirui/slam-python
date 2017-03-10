#!/usr/bin/python
#-*- coding:utf-8 -*-

import cv2
import numpy as np

width = 1240
height = 1754

row = []
col = []

for rows in range(0, width):
	for cols in range(0, height):
		if (76 < cols < 1677) and (19 < rows < 1220):
			if (((cols - 77) / 200) % 2 != 0) and (((rows - 20) / 200) % 2 != 0):
				col.append([0, 0, 0])
			if (((cols - 77) / 200) % 2 == 0) and (((rows - 20) / 200) % 2 == 0):
				col.append([0, 0, 0])
			if (((cols - 77) / 200) % 2 != 0) and (((rows - 20) / 200) % 2 == 0):
				col.append([255, 255, 255])
			if (((cols - 77) / 200) % 2 == 0) and (((rows - 20) / 200) % 2 != 0):
				col.append([255, 255, 255])
		else:
			col.append([255, 255, 255])
	row.append(col)
	col = []

img = np.array(row, dtype = np.uint8)
cv2.imwrite( "Checkerboard.png", img )
