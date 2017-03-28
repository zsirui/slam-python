#!/usr/bin/env python
# -*- coding: utf-8 -*-

import slamBase
import pcl
import readyaml
from numpy import array, float32, pi
from os import path
from cv2 import norm

RGBFileNamePath = './data/rgb_png'
DepthFileNamePath = './data/depth_png'
CalibrationDataFile = './calibration/asus/camera.yml'
CloudFilename = './data/cloud.pcd'
# 点云分辨率
GRIDSIZE = 0.02
# 起始与终止索引
START_INDEX = 1
END_INDEX = 700
# 最小匹配数量
MIN_GOOD_MATCH = 10
# 最小内点
MIN_INLIERS = 5
# 最大运动误差
MAX_NORM = 0.3

C = array([[518.0,0,325.0],[0,519.0,253.5],[0,0,1]])

def normofTransform(rvec, tvec):
	return abs(min(norm(rvec), pi * 2 - norm(rvec))) + abs(norm(tvec))

def joinPointCloud(pointCloud, color, RGBFileName, DepthFileName, lastFrame, CameraIntrinsicData, DistortionCoefficients, camera):
	currentFrame = slamBase.Frame(RGBFileName, DepthFileName)
	pnp = slamBase.SolvePnP(DistortionCoefficients, CameraIntrinsicData, camera, lastFrame, currentFrame)

	try:
		len(pnp.inliers)
	except:
		pass
	else:
		if len(pnp.inliers) < MIN_INLIERS:
			pass
		else:
			Norm = normofTransform(pnp.rvec, pnp.tvec)
			print('norm = ' + str(Norm))
			if Norm >= MAX_NORM:
				pass
			else:
				p0, c0 = slamBase.imageToPointCloud(RGBFileName, DepthFileName, camera)
				colors = color + c0
				p = slamBase.transformPointCloud(p0, pnp.T)
				pointCloud = slamBase.addPointCloud(pointCloud.to_list(), p)
				voxel = pointCloud.make_voxel_grid_filter()
				voxel.set_leaf_size( GRIDSIZE, GRIDSIZE, GRIDSIZE )
				pointCloud = voxel.filter()
				lastFrame = currentFrame
		return pointCloud, lastFrame, colors

def visualOdometry():
	CameraIntrinsicData, DistortionCoefficients = readyaml.parseYamlFile(CalibrationDataFile)
	DistortionCoefficients = array([0,0,0,0], dtype = float32)
	camera = slamBase.CameraIntrinsicParameters(C[0][2], C[1][2], C[0][0], C[1][1], 1000.0)
	frame = slamBase.Frame(path.join(RGBFileNamePath, str(START_INDEX) + '.png'), path.join(DepthFileNamePath, str(START_INDEX) + '.png'))
	p, colors = slamBase.imageToPointCloud(path.join(RGBFileNamePath, str(START_INDEX) + '.png'), path.join(DepthFileNamePath, str(START_INDEX) + '.png'), camera)
	pcd = pcl.PointCloud()
	pcd.from_array(array(p, dtype = float32))
	for i in range(START_INDEX, END_INDEX):
		try:
			pcd, frame, colors = joinPointCloud(pcd, colors, path.join(RGBFileNamePath, str(i + 1) + '.png'), path.join(DepthFileNamePath, str(i + 1) + '.png'), frame, C, DistortionCoefficients, camera)
		except:
			pass
		
	pcl.save(pcd, CloudFilename, format = 'pcd')
	# slamBase.addColorToPCDFile(CloudFilename, colors)

if __name__ == '__main__':
	visualOdometry()
