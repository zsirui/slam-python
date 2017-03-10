#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pcl
import readyaml

class CameraIntrinsicParameters(object):
	"""docstring for CameraIntrinsicParameters"""
	def __init__(self, cx, cy, fx, fy, scale):
		super(CameraIntrinsicParameters, self).__init__()
		self.cx = cx
		self.cy = cy
		self.fx = fx
		self.fy = fy
		self.scale = scale

def addColorToPCDFile(filename, colors):
	with open(filename, 'rb') as f:
		lines = f.readlines()
	lines[2] = lines[2].split('\n')[0] + ' rgb\n'
	lines[3] = lines[3].split('\n')[0] + ' 4\n'
	lines[4] = lines[4].split('\n')[0] + ' I\n'
	lines[5] = lines[5].split('\n')[0] + ' 1\n'
	for i in range(11, len(colors) + 11):
		lines[i] = lines[i].split('\n')[0] + ' ' + str(colors[i - 11]) + '\n'
	with open(filename, 'wb') as fw:
		fw.writelines(lines)

def point2dTo3d(n, m, d, camera):
	z = float(d) / camera.scale
	x = (n - camera.cx) * z / camera.fx
	y = (m - camera.cy) * z / camera.fy
	point = np.array([x, y, z], dtype = np.float32)
	return point

def imageToPointCloud(RGBFilename, DepthFilename, CloudFilename, camera):
	rgb = cv2.imread( RGBFilename )
	depth = cv2.imread( DepthFilename, cv2.COLOR_BGR2GRAY )
	# ROS中rqt保存的深度摄像头的图片是rgb格式，需要转换成单通道灰度格式
	if len(depth[0][0]) == 3:
		depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)

	cloud = pcl.PointCloud()
	rows = len(depth)
	cols = len(depth[0])
	pointcloud = []
	colors = []
	for m in range(0, rows):
		for n in range(0, cols):
			d = depth[m][n]
			if d == 0:
				pass
			else:
				point = point2dTo3d(n, m, d, camera)
				pointcloud.append(point)
				b = rgb[m][n][0]
				g = rgb[m][n][1]
				r = rgb[m][n][2]
				color = (r << 16) | (g << 8) | b
				colors.append(int(color))
	pointcloud = np.array(pointcloud, dtype = np.float32)
	cloud.from_array(pointcloud)
	pcl.save(cloud, CloudFilename, format = 'pcd')
	addColorToPCDFile(CloudFilename, colors)

if __name__ == '__main__':
	CameraIntrinsicData = readyaml.parseYamlFile('./calibration_data/tianmao/camera.yml')
	camera = CameraIntrinsicParameters(CameraIntrinsicData[0][2], CameraIntrinsicData[1][2], CameraIntrinsicData[0][0], CameraIntrinsicData[1][1], 1000.0)
	imageToPointCloud('rgb.png', 'depth.png', 'cloud1.pcd', camera)
