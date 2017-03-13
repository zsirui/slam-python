#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from slamBase import point2dTo3d, CameraIntrinsicParameters
import readyaml

def readImgFiles(RGBFilenames, DepthFilenames, paras):
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

def computeMatches(rgb1, rgb2, depth1, depth2, CameraIntrinsicData, distCoeffs, camera):
	sift = cv2.xfeatures2d.SIFT_create()
	kp1, des1 = sift.detectAndCompute(rgb1, None)
	kp2, des2 = sift.detectAndCompute(rgb2, None)
	print("Key points of two images: " + str(len(kp1)) + ", " + str(len(kp2)))

	imgShow = None
	imgShow = cv2.drawKeypoints(rgb1, kp1, imgShow, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	cv2.imshow( "keypoints_1", imgShow )
	cv2.imwrite( "./data/keypoints_1.png", imgShow )
	cv2.waitKey(0)

	imgShow = None
	imgShow = cv2.drawKeypoints(rgb2, kp2, imgShow, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	cv2.imshow( "keypoints_2", imgShow )
	cv2.imwrite( "./data/keypoints_2.png", imgShow )
	cv2.waitKey(0)

	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks = 50)   # or pass empty dictionary
	matcher = cv2.FlannBasedMatcher(index_params, search_params)
	matches = matcher.match(des1, des2)
	print("Find total " + str(len(matches)) + " matches.")

	imgMatches = None
	imgMatches = cv2.drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches )
	cv2.imshow( "matches", imgMatches )
	cv2.imwrite( "./data/matches.png", imgMatches )
	cv2.waitKey(0)

	goodMatches = []
	minDis = 9999.0
	for i in range(0, len(matches)):
		if matches[i].distance < minDis:
			minDis = matches[i].distance
	for i in range(0, len(matches)):
		if matches[i].distance < (minDis * 4):
			goodMatches.append(matches[i])
	print("good matches = " + str(len(goodMatches)))

	imgMatches = None
	imgMatches = cv2.drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches )
	cv2.imshow( "good_matches", imgMatches )
	cv2.imwrite( "./data/good_matches.png", imgMatches )
	cv2.waitKey(0)

	pts_obj = []
	pts_img = []

	for i in range(0, len(goodMatches)):
		p = kp1[goodMatches[i].queryIdx].pt
		d = depth1[int(p[1])][int(p[0])]
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
	retval, rvec, tvec, inliers = cv2.solvePnPRansac( pts_obj, pts_img, cameraMatrix, distCoeffs, useExtrinsicGuess = False, iterationsCount = 100, reprojectionError = 0.66 )
	print("inliers: " + str(len(inliers)))
	print("R=" + str(rvec))
	print("t=" + str(tvec))

	matchesShow = []
	for i in range(0, len(inliers)):
		matchesShow.append( goodMatches[inliers[i][0]] )
	imgMatches = None
	imgMatches = cv2.drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches )
	cv2.imshow( "inlier matches", imgMatches )
	cv2.imwrite( "./data/inliers.png", imgMatches )
	cv2.waitKey(0)

def main():
	rgbfilenames = ('./data/rgb1.png', './data/rgb2.png')
	depthfilenames = ('./data/depth1.png', './data/depth2.png')
	CameraIntrinsicData, DistortionCoefficients = readyaml.parseYamlFile('./calibration_data/asus/camera.yml')
	camera = CameraIntrinsicParameters(CameraIntrinsicData[0][2], CameraIntrinsicData[1][2], CameraIntrinsicData[0][0], CameraIntrinsicData[1][1], 1000.0)
	rgbs, depths = readImgFiles(rgbfilenames, depthfilenames, cv2.COLOR_BGR2GRAY)
	computeMatches(rgbs[0], rgbs[1], depths[0], depths[1], CameraIntrinsicData, DistortionCoefficients, camera)

if __name__ == '__main__':
	main()
