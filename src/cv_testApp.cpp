/*
*
* Copyright (c) 2013, Ban the Rewind
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following
* conditions are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
*
* Neither the name of the Ban the Rewind nor the names of its
* contributors may be used to endorse or promote products
* derived from this software without specific prior written
* permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#define KINECT_WIDTH 320
#define KINECT_HEIGHT 240
// Includes
#include <algorithm>
#include "boost/algorithm/string.hpp"
#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

#include "CinderOpenCv.h"

/*
* This application explores the features of the Kinect SDK wrapper. It
* demonstrates how to start a device, query for devices, adjust tilt,
* and read and represent color, depth, and skeleton data.
* It's also useful as a device test and control panel.
*/

class cv_testApp : public ci::app::AppBasic
{
public:
	void								draw();
	void								prepareSettings(ci::app::AppBasic::Settings *settings);
	void								setup();
	void								shutdown();
	void								update();
	void								keyDown(ci::app::KeyEvent e);
private:
	

	// Capturing flags
	bool								mCapture;
	bool								mCapturePrev;
	bool								mBinaryMode;
	bool								mBinaryModePrev;
	bool								mEnabledColor;
	bool								mEnabledColorPrev;
	bool								mEnabledDepth;
	bool								mEnabledDepthPrev;
	bool								mEnabledNearMode;
	bool								mEnabledNearModePrev;
	bool								mEnabledSeatedMode;
	bool								mEnabledSeatedModePrev;
	bool								mEnabledSkeletons;
	bool								mEnabledSkeletonsPrev;
	bool								mEnabledStats;
	bool								mFlipped;
	bool								mFlippedPrev;
	bool								mInverted;
	bool								mInvertedPrev;

	// Kinect
	ci::Surface8u						mColorSurface;
	ci::Surface16u						mDepthSurface;
	std::vector<ci::Vec3f>				mHandPos;
	std::vector<cv::Mat>				mHandsMat;
	ci::gl::Texture						mHandsTexture;
	int32_t								mDeviceCount;
	KinectSdk::DeviceOptions			mDeviceOptions;
	KinectSdk::KinectRef				mKinect;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	int32_t								mTilt;
	int32_t								mTiltPrev;
	int32_t								mUserCount;
	void								startKinect();

	// Kinect callbacks
	int32_t								mCallbackDepthId;
	int32_t								mCallbackSkeletonId;
	int32_t								mCallbackColorId;
	void								onColorData(ci::Surface8u surface, const KinectSdk::DeviceOptions& deviceOptions);
	void								onDepthData(ci::Surface16u surface, const KinectSdk::DeviceOptions& deviceOptions);
	void								onSkeletonData(std::vector<KinectSdk::Skeleton> skeletons, const KinectSdk::DeviceOptions& deviceOptions);

	// Camera
	ci::CameraPersp						mCamera;

	// Params
	float								mFrameRateApp;
	float								mFrameRateColor;
	float								mFrameRateDepth;
	float								mFrameRateSkeletons;
	bool								mFullScreen;
	ci::params::InterfaceGlRef			mParams;
	bool								mRemoveBackground;
	bool								mRemoveBackgroundPrev;
	void								resetStats();

	// Save screen shot
	void								screenShot();

	//enum
	enum mode{DEPTH, VIDEO} displayMode;
};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Render
void cv_testApp::draw()
{
	// Clear window
	gl::setViewport(getWindowBounds());
	gl::clear(Colorf::gray(0.1f));

	// We're capturing
	if (mKinect->isCapturing()) {
		// Switch to 2D
		gl::pushMatrices();
		gl::setMatricesWindow(getWindowSize(), true);
		gl::scale(Vec2f::one() * 3);
		
		// Draw depth and color textures
		//Rectf destRect(0, 0, KINECT_WIDTH, KINECT_HEIGHT);

		switch (displayMode)
		{
		case cv_testApp::DEPTH:
			//Area srcArea(0, 0, mDepthSurface.getWidth(), mDepthSurface.getHeight());
			if (mDepthSurface)
				gl::draw(gl::Texture(mDepthSurface));
			if (!mHandsMat.empty()){
				for (size_t i = 0; i < mHandsMat.size(); i++){
					gl::pushMatrices();
					gl::translate(mHandPos[i].xy() - (Vec2f::one() * (1 / mHandPos[i].z) * 40));
					gl::draw(gl::Texture(fromOcv(mHandsMat[i])));
					gl::popMatrices();
				}
			}
			break;
		case cv_testApp::VIDEO:
			//Area srcArea(0, 0, mColorSurface.getWidth(), mColorSurface.getHeight());
			
			gl::draw(gl::Texture(mColorSurface));
			break;
		default:
			break;
		}
		
		// Set up camera for 3D
		//gl::setMatrices(mCamera);

		// Move skeletons down below the rest of the interface

		//gl::color(1, 0, 0);
		// Iterate through skeletons
		//for (size_t i = 0; i < mHandPos.size(); i++){
		//	gl::drawStrokedCircle(mHandPos[i].xy(), (1 / mHandPos[i].z)*40, 16);
		//}	
		//gl::color(1, 1, 1);

		gl::popMatrices();
	}

	// Draw the interface
	mParams->draw();
}

// Receives color data
void cv_testApp::onColorData(Surface8u surface, const DeviceOptions& deviceOptions)
{
	mColorSurface = surface;
}

// Receives depth data
void cv_testApp::onDepthData(Surface16u surface, const DeviceOptions& deviceOptions)
{
	mDepthSurface = surface;
}

// Receives skeleton data
void cv_testApp::onSkeletonData(vector<Skeleton> skeletons, const DeviceOptions& deviceOptions)
{
	mSkeletons = skeletons;
}

// Prepare window
void cv_testApp::prepareSettings(Settings *settings)
{
	settings->setWindowSize(KINECT_WIDTH * 3, KINECT_HEIGHT * 3);
	settings->setFrameRate(60.0f);
}

// Reset statistics
void cv_testApp::resetStats()
{
	mFrameRateDepth = 0.0f;
	mFrameRateSkeletons = 0.0f;
	mFrameRateColor = 0.0f;
	mUserCount = 0;
}

// Take screen shot
void cv_testApp::screenShot()
{
	writeImage(getAppPath() / fs::path("frame" + toString(getElapsedFrames()) + ".png"), copyWindowSurface());
}

// Set up
void cv_testApp::setup()
{
	// Set up OpenGL
	glLineWidth(2.0f);
	gl::color(ColorAf::white());

	// Set up camera
	mCamera.lookAt(Vec3f(0.0f, 0.0f, 3.0f), Vec3f::zero());
	mCamera.setPerspective(45.0f, getWindowAspectRatio(), 1.0f, 1000.0f);

	// Initialize parameters
	mBinaryMode = false;
	mBinaryModePrev = mBinaryMode;
	mCapture = true;
	mCapturePrev = mCapture;
	mDeviceCount = 0;
	mEnabledColor = true;
	mEnabledColorPrev = mEnabledColor;
	mEnabledDepth = true;
	mEnabledDepthPrev = mEnabledDepth;
	mEnabledNearMode = false;
	mEnabledNearModePrev = mEnabledNearMode;
	mEnabledSeatedMode = false;
	mEnabledSeatedModePrev = mEnabledSeatedMode;
	mEnabledSkeletons = true;
	mEnabledSkeletonsPrev = mEnabledSkeletons;
	mEnabledStats = true;
	mFlipped = false;
	mFlippedPrev = mFlipped;
	mFrameRateApp = 0.0f;
	mFrameRateDepth = 0.0f;
	mFrameRateSkeletons = 0.0f;
	mFrameRateColor = 0.0f;
	mFullScreen = isFullScreen();
	mInverted = false;
	mInvertedPrev = mInverted;
	mRemoveBackground = true;
	mRemoveBackgroundPrev = mRemoveBackground;
	mUserCount = 0;
	displayMode = DEPTH;


	// Start image capture
	mKinect = Kinect::create();
	startKinect();

	// Add callbacks
	mCallbackDepthId = mKinect->addDepthCallback(&cv_testApp::onDepthData, this);
	mCallbackSkeletonId = mKinect->addSkeletonTrackingCallback(&cv_testApp::onSkeletonData, this);
	mCallbackColorId = mKinect->addColorCallback(&cv_testApp::onColorData, this);

	// Setup the parameters
	mParams = params::InterfaceGl::create(getWindow(), "Parameters", toPixels(Vec2i(245, 500)));
	mParams->addText("DEVICE");
	mParams->addParam("Device count", &mDeviceCount, "", true);
	mParams->addParam("Device angle", &mTilt, "min=-" +
		toString(Kinect::MAXIMUM_TILT_ANGLE) + " max=" + toString(Kinect::MAXIMUM_TILT_ANGLE) + " step=1");
	mParams->addSeparator();
	mParams->addText("STATISTICS");
	mParams->addParam("Collect statistics", &mEnabledStats, "key=t");
	mParams->addParam("App frame rate", &mFrameRateApp, "", true);
	mParams->addParam("Color frame rate", &mFrameRateColor, "", true);
	mParams->addParam("Depth frame rate", &mFrameRateDepth, "", true);
	mParams->addParam("Skeleton frame rate", &mFrameRateSkeletons, "", true);
	mParams->addParam("User count", &mUserCount, "", true);
	mParams->addSeparator();
	mParams->addText("CAPTURE");
	mParams->addParam("Capture", &mCapture, "key=c");
	mParams->addParam("Depth", &mEnabledDepth, "key=d");
	mParams->addParam("Skeletons", &mEnabledSkeletons, "key=k");
	mParams->addParam("Color", &mEnabledColor, "key=v");
	mParams->addSeparator();
	mParams->addText("INPUT");
	mParams->addParam("Remove background", &mRemoveBackground, "key=b");
	mParams->addParam("Binary depth mode", &mBinaryMode, "key=w");
	mParams->addParam("Invert binary image", &mInverted, "key=i");
	mParams->addParam("Flip input", &mFlipped, "key=m");
	mParams->addParam("Near mode", &mEnabledNearMode, "key=n");
	mParams->addParam("Seated mode", &mEnabledSeatedMode, "key=e");
	mParams->addSeparator();
	mParams->addText("APPLICATION");
	mParams->addParam("Full screen", &mFullScreen, "key=f");
	mParams->addButton("Screen shot", bind(&cv_testApp::screenShot, this), "key=s");
	mParams->addButton("Quit", bind(&cv_testApp::quit, this), "key=q");
}

// Quit
void cv_testApp::shutdown()
{
	// Stop input
	mKinect->removeCallback(mCallbackDepthId);
	mKinect->removeCallback(mCallbackSkeletonId);
	mKinect->removeCallback(mCallbackColorId);
	mKinect->stop();
	mSkeletons.clear();
}

// Start Kinect input
void cv_testApp::startKinect()
{
	// Update device count
	mDeviceCount = Kinect::getDeviceCount();

	// Configure device
	mDeviceOptions.enableDepth(mEnabledDepth);
	mDeviceOptions.enableNearMode(mEnabledNearMode);
	mDeviceOptions.enableSkeletonTracking(mEnabledSkeletons, mEnabledSeatedMode);
	mDeviceOptions.enableColor(mEnabledColor);
	mKinect->enableBinaryMode(mBinaryMode);
	mKinect->removeBackground(mRemoveBackground);
	mKinect->setFlipped(mFlipped);

	// Stop, if capturing
	if (mKinect->isCapturing()) {
		mKinect->stop();
	}

	// Start Kinect
	mKinect->start(mDeviceOptions);

	// Trace out the unique device ID
	console() << "Device ID: " << mKinect->getDeviceOptions().getDeviceId() << endl;

	// Get device angle angle
	mTilt = mKinect->getTilt();
	mTiltPrev = mTilt;

	// Clear stats
	resetStats();
}

// Runs update logic
void cv_testApp::update()
{
	// Update frame rate
	mFrameRateApp = getAverageFps();

	// Toggle fullscreen
	if (mFullScreen != isFullScreen()) {
		setFullScreen(mFullScreen);
	}

	// Toggle background remove
	if (mRemoveBackground != mRemoveBackgroundPrev) {
		mKinect->removeBackground(mRemoveBackground);
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	// Toggle mirror image
	if (mFlipped != mFlippedPrev) {
		mKinect->setFlipped(mFlipped);
		mFlippedPrev = mFlipped;
	}

	// Toggle capture
	if (mCapture != mCapturePrev) {
		mCapturePrev = mCapture;
		if (mCapture) {
			startKinect();
		}
		else {
			mKinect->stop();
		}
	}

	// Toggle input tracking types (requires device restart)
	if (mEnabledColor != mEnabledColorPrev ||
		mEnabledDepth != mEnabledDepthPrev ||
		mEnabledNearMode != mEnabledNearModePrev ||
		mEnabledSeatedMode != mEnabledSeatedModePrev ||
		mEnabledSkeletons != mEnabledSkeletonsPrev) {
		startKinect();
		mEnabledColorPrev = mEnabledColor;
		mEnabledDepthPrev = mEnabledDepth;
		mEnabledNearModePrev = mEnabledNearMode;
		mEnabledSeatedModePrev = mEnabledSeatedMode;
		mEnabledSkeletonsPrev = mEnabledSkeletons;
	}

	// Toggle binary mode
	if (mBinaryMode != mBinaryModePrev ||
		mInverted != mInvertedPrev) {
		mKinect->enableBinaryMode(mBinaryMode, mInverted);
		mBinaryModePrev = mBinaryMode;
		mInvertedPrev = mInverted;
	}

	// Check if device is capturing
	if (mKinect->isCapturing()) {

		// Update device
		mKinect->update();

		// Adjust Kinect camera angle, as needed
		if (mTilt != mTiltPrev) {
			mKinect->setTilt(mTilt);
			mTiltPrev = mTilt;
		}

		// Statistics enabled (turn off to improve performance)
		if (mEnabledStats) {

			// Update user count
			mUserCount = mKinect->getUserCount();

			// Update frame rates
			mFrameRateColor = mKinect->getColorFrameRate();
			mFrameRateDepth = mKinect->getDepthFrameRate();
			mFrameRateSkeletons = mKinect->getSkeletonFrameRate();

		}
		else {

			// Clear stats
			resetStats();

		}

	}
	else {

		// If Kinect initialization failed, try again every 90 frames
		if (getElapsedFrames() % 90 == 0) {
			mKinect->start();
		}

	}

	// get hands position 
	mHandPos.clear();
	mHandsMat.clear();
	cv::Mat img_depth;
	if (mDepthSurface)
		img_depth = toOcv(Channel8u(mDepthSurface));

	for (vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt) {
		vector<JointName> jointList;
		jointList.push_back(NUI_SKELETON_POSITION_HAND_LEFT);
		jointList.push_back(NUI_SKELETON_POSITION_HAND_RIGHT);

		// Draw bones and joints
		for (size_t m = 0; m < jointList.size(); m++) {

			// Get positions of each hand in this bone to draw it
			if (!skeletonIt->empty()){
				const Bone& bone = skeletonIt->at(jointList[m]);
				Vec2f handPos2d = mKinect->getSkeletonDepthPos(bone.getPosition());
				float handDepth = bone.getPosition().z;

				// Draw joint
				Vec3f handPos3d(handPos2d, handDepth);
				mHandPos.push_back(handPos3d);
				// get hands from depth surface
				float radius = (1 / handDepth) * 40;
				if (mDepthSurface){
					if (handPos2d.x > radius
						&& handPos2d.x < 320 - radius
						&& handPos2d.y > radius
						&& handPos2d.y < 240 - radius)
					{
						// Hand thresholding
						double hand_intensity = img_depth.at<uchar>(handPos2d.y, handPos2d.x);
						
						int rectTopLeftX = handPos2d.x - radius;
						int rectTopLeftY = handPos2d.y - radius;

						cv::Rect handRect(rectTopLeftX, rectTopLeftY, radius * 2, radius * 2);
						cv::Mat handMat = img_depth(handRect);

						cv::Mat img_thresh, img_canny;
						cv::threshold(handMat, img_thresh, hand_intensity + 1, 255, CV_THRESH_BINARY_INV);
						cv::medianBlur(img_thresh, img_thresh, 5);

						// Find contour
						Canny(img_thresh, img_canny, 60, 110);

						vector< vector<cv::Point> > contours;
						cv::findContours(img_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

						vector< vector<cv::Point> > hull(contours.size());
						vector< vector<int> > hullI(contours.size());

						for (size_t i = 0; i < contours.size(); i++)
						{
							cv::convexHull(contours[i], hull[i], false);
							cv::convexHull(contours[i], hullI[i], false);
						}

						//draw Contours
						cv::Mat drawing = cv::Mat::zeros(img_thresh.size(), CV_8UC3);
						for (size_t i = 0; i < contours.size(); i++)
						{
							cv::Scalar color = cv::Scalar(255, 255, 255);
							cv::drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, cv::Point());
							cv::drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, cv::Point());
						}

						// get hull points of fingertips
						vector< vector<cv::Point> > fhull, finhull;
						int fhullIndex = -1;
						cv::Point p1, p2, hullP;
						double th1, th2, th;

						for (size_t i = 0; i < contours.size(); i++)
						{
							if (contours[i].size() < 50)
							{
								continue;
							}
							fhull.push_back(vector<cv::Point>());
							finhull.push_back(vector<cv::Point>());
							fhullIndex++;

							for (size_t j = 0; j < hullI[i].size(); j++)
							{
								hullP = contours[i][hullI[i][j]];
								p1 = contours[i][(hullI[i][j] - 16) % contours[i].size()];
								p2 = contours[i][(hullI[i][j] + 16) % contours[i].size()];

								th1 = abs(atan2(hullP.y - p1.y, hullP.x - p1.x) * 180 / CV_PI);
								th2 = abs(atan2(hullP.y - p2.y, hullP.x - p2.x) * 180 / CV_PI);
								th = abs(th1 - th2);
								if (th < 40)
								{
									finhull[fhullIndex].push_back(hullP);
								}
							}
						}

						// get exact fingertips
						vector< vector<cv::Point> >nfhull;
						int chullx, chully, add_flag = 1, nhullindex = -1;

						for (size_t i = 0; i<finhull.size(); i++)
						{
							nfhull.push_back(vector<cv::Point>());
							for (size_t j = 0; j < finhull[i].size(); j++)
							{
								chullx = finhull[i][j].x;
								chully = finhull[i][j].y;
								add_flag = 1;
								for (size_t k = 0; k<nfhull[i].size(); k++)
								{
									if (nfhull[i][k].x > chullx - 15 && nfhull[i][k].x < chullx + 15
										&& nfhull[i][k].y > chully - 15 && nfhull[i][k].y < chully + 15)
									{
										add_flag = 0;
										break;
									}
								}
								if (add_flag)
								{
									nfhull[i].push_back(cv::Point(chullx, chully));
								}
							}
						}

						// draw hull Points
						for (size_t i = 0; i<nfhull.size(); i++)
						{
							for (size_t j = 0; j<nfhull[i].size(); j++)
							{
								cv::circle(drawing, nfhull[i][j], 7, (0, 255, 255), 3, 8);
							}
						}
						mHandsMat.push_back(drawing);
					}

				}
			}

		}
	}
	
}

void cv_testApp::keyDown(KeyEvent e){
	if (e.getChar() == '1'){
		displayMode = DEPTH;
	}
	else if (e.getChar() == '2'){
		displayMode = VIDEO;
	}
}


// Run application
CINDER_APP_BASIC(cv_testApp, RendererGl)
