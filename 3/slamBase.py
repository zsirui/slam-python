#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pcl

class CameraIntrinsicParameters(object):
	"""docstring for CameraIntrinsicParameters"""
	def __init__(self, cx, cy, fx, fy, scale):
		super(CameraIntrinsicParameters, self).__init__()
		self.cx = cx
		self.cy = cy
		self.fx = fx
		self.fy = fy
		self.scale = scale

class SolvePnP(object):
	"""docstring for SolvePnP"""
	def __init__(self, RGBFileNameList, DepthFileNameList, distCoeffs, CameraIntrinsicData, camera):
		super(SolvePnP, self).__init__()
		self.RGBFileNameList = RGBFileNameList
		self.DepthFileNameList = DepthFileNameList
		self.distCoeffs = distCoeffs
		self.CameraIntrinsicData = CameraIntrinsicData
		self.camera = camera
		self.rgbs, self.depths = self.readImgFiles(self.RGBFileNameList, self.DepthFileNameList, cv2.COLOR_BGR2GRAY)
		self.rvec, self.tvec, self.inliers = self.ResultOfPnP(self.rgbs[0], self.rgbs[1], self.depths, self.distCoeffs, self.CameraIntrinsicData, self.camera)
		self.T = self.transformMatrix(self.rvec, self.tvec)

	def transformMatrix(self, rvec, tvec):
		temp = np.matrix([0, 0, 0, 1])
		r = np.matrix(rvec)
		t = np.matrix(tvec)
		dst, jacobian = cv2.Rodrigues(r)
		c = np.hstack((dst, t))
		T = np.vstack((c, temp))
		return T

	def readImgFiles(self, RGBFilenames, DepthFilenames, paras):
		rgbs = []
		depths = []
		for i in range(0, len(RGBFilenames)):
			rgbs.append(cv2.imread(RGBFilenames[i]))
		for j in range(0, len(DepthFilenames)):
			depth = cv2.imread(DepthFilenames[i], paras)
			# ROS中rqt保存的深度摄像头的图片是rgb格式，需要转换成单通道灰度格式
			if len(depth[0][0]) == 3:
				depths.append(cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY))
			else:
				depths.append(depth)
		return rgbs, depths

	def ResultOfPnP(self, rgb1, rgb2, depth, distCoeffs, CameraIntrinsicData, camera):
		sift = cv2.xfeatures2d.SIFT_create()
		kp1, des1 = sift.detectAndCompute(rgb1, None)
		kp2, des2 = sift.detectAndCompute(rgb2, None)

		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)   # or pass empty dictionary
		matcher = cv2.FlannBasedMatcher(index_params, search_params)
		matches = matcher.match(des1, des2)
		print("Find total " + str(len(matches)) + " matches.")

		goodMatches = []
		minDis = 9999.0
		for i in range(0, len(matches)):
			if matches[i].distance < minDis:
				minDis = matches[i].distance
		for i in range(0, len(matches)):
			if matches[i].distance < (minDis * 4):
				goodMatches.append(matches[i])
		print("good matches = " + str(len(goodMatches)))

		pts_obj = []
		pts_img = []

		for i in range(0, len(goodMatches)):
			p = kp1[goodMatches[i].queryIdx].pt
			d = depth[0][int(p[1])][int(p[0])]
			if d == 0:
				pass
			else:
				pts_img.append(kp2[goodMatches[i].trainIdx].pt)
				pd = point2dTo3d(p[0], p[1], d, camera)
				pts_obj.append(pd)
		pts_obj = np.array(pts_obj)
		pts_img = np.array(pts_img)

		cameraMatrix = np.matrix(CameraIntrinsicData)
		rvec = None
		tvec = None
		inliers = None
		retval, rvec, tvec, inliers = cv2.solvePnPRansac( pts_obj, pts_img, cameraMatrix, distCoeffs, useExtrinsicGuess = False, iterationsCount = 100, reprojectionError = 1.76 )
		return rvec, tvec, inliers

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

def transformPointCloud(src, T):
	pointcloud = []
	out = pcl.PointCloud()
	for item in src:
		a = list(item)
		a.append(1)
		a = np.matrix(a)
		a = a.reshape((-1, 1))
		temp = T * a
		temp = temp.reshape((1, -1))
		temp = np.array(temp)
		temp = list(temp[0])
		pointcloud.append(temp[0:3])
	return pointcloud

def addPointCloud(cloud1, cloud2):
	cloud = cloud1 + cloud2
	cloud = np.array(cloud, dtype = np.float32)
	out = pcl.PointCloud()
	out.from_array(cloud)
	return out

def point2dTo3d(n, m, d, camera):
	z = float(d) / camera.scale
	x = (n - camera.cx) * z / camera.fx
	y = (m - camera.cy) * z / camera.fy
	point = np.array([x, y, z], dtype = np.float32)
	return point

def imageToPointCloud(RGBFilename, DepthFilename, camera):
	rgb = cv2.imread( RGBFilename )
	depth = cv2.imread( DepthFilename, cv2.COLOR_BGR2GRAY )
	# ROS中rqt保存的深度摄像头的图片是rgb格式，需要转换成单通道灰度格式
	if len(depth[0][0]) == 3:
		depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)

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
	return pointcloud, colors
