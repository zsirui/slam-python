#!/usr/bin/env python
# -*- coding: utf-8 -*-

import slamBase
from pcl import save
import readyaml

RGBFileNameList = ['./data/rgb1.png', './data/rgb2.png']
DepthFileNameList = ['./data/depth1.png', './data/depth2.png']
CalibrationDataFile = './calibration/asus/camera.yml'
CloudFilename = './data/cloud.pcd'

def main():
	CameraIntrinsicData, DistortionCoefficients = readyaml.parseYamlFile(CalibrationDataFile)
	camera = slamBase.CameraIntrinsicParameters(CameraIntrinsicData[0][2], CameraIntrinsicData[1][2], CameraIntrinsicData[0][0], CameraIntrinsicData[1][1], 1000.0)
	frame1 = slamBase.Frame(RGBFileNameList[0], DepthFileNameList[0])
	frame2 = slamBase.Frame(RGBFileNameList[1], DepthFileNameList[1])
	pnp = slamBase.SolvePnP(DistortionCoefficients, CameraIntrinsicData, camera, frame1, frame2)

	p0, c0 = slamBase.imageToPointCloud(RGBFileNameList[0], DepthFileNameList[0], camera)
	p1, c1 = slamBase.imageToPointCloud(RGBFileNameList[1], DepthFileNameList[1], camera)
	colors = c0 + c1
	p = slamBase.transformPointCloud(p0, pnp.T)
	pointcloud = slamBase.addPointCloud(p, p1)
	save(pointcloud, CloudFilename, format = 'pcd')
	slamBase.addColorToPCDFile(CloudFilename, colors)

if __name__ == '__main__':
	main()
