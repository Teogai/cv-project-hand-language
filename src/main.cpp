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
#include "cinder/Text.h"
#include "Kinect.h"

#include "CinderOpenCv.h"
#include <fstream>
#include <algorithm>

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
	bool								mUserColor;
	bool								mPrintGestureData;
	std::string							mLabel;
	int									mKvalue;
	bool								mDebug;

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
	bool								mUserColorPrev;
	void								resetStats();

	// Save screen shot
	void								screenShot();	

	//enum
	enum mode{DEPTH, VIDEO} displayMode;

	//data
	struct Gesture{
		std::vector<double> hu;
		std::string name;

		Gesture(std::vector<double> hu, std::string name){
			this->hu = hu;
			this->name = name;
		}
	};
	std::vector< std::vector<Gesture> >			mGestures;

	// For min-max normalization
	std::vector<double>						huMin;
	std::vector<double>						huMax;

	// K-NN
	std::string								kNN(int defect, Gesture gesture, int k);
	// Keeping histories of each detected joint
	std::vector<std::string>				gestureHistory;
	std::string								subTitle;
	std::string								lastSubTitle;

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
		gl::setMatricesWindow(getWindowSize(), true);


		// Draw depth and color textures
		//Rectf destRect(0, 0, KINECT_WIDTH, KINECT_HEIGHT);

		if (mDebug)
		{
			gl::pushMatrices();
			gl::scale(Vec2f::one() * 3);
			if (mDepthSurface)
				gl::draw(gl::Texture(mDepthSurface));
			if (!mHandsMat.empty()){
				for (size_t i = 0; i < mHandsMat.size(); i++){					
					gl::translate(mHandPos[i].xy() - (Vec2f::one() * (1 / mHandPos[i].z) * 40));
					gl::draw(gl::Texture(fromOcv(mHandsMat[i])));					
				}
			}
			gl::popMatrices();
		}
		else{
			if (mColorSurface){
				gl::pushMatrices();
				gl::scale(Vec2f::one() * 3);
				gl::scale(Vec2f(0.5f, 0.5f));
				gl::draw(gl::Texture(mColorSurface));
				gl::popMatrices();
			}		

			if (subTitle != ""){
				gl::enableAlphaBlending();
				TextBox tbox = TextBox().alignment(TextBox::CENTER).font(Font("TAHOMA", 80.0f)).size(Vec2i(getWindowWidth(), getWindowHeight() / 4)).text(subTitle);
				tbox.setColor(Color(1.0f, 1.0f, 1.0f));
				tbox.setBackgroundColor(ColorA(0, 0, 0, 0));
				// Shadow is drawn first
				gl::color(0, 0, 0);
				gl::draw(gl::Texture::create(tbox.render()), Vec2i(3, 3 + 3 * getWindowHeight() / 4));
				gl::color(1, 1, 1);
				// Then real text
				gl::draw(gl::Texture::create(tbox.render()), Vec2i(0, 3 * getWindowHeight() / 4));
				
				//gl::drawStringCentered(subTitle, Vec2f(getWindowWidth()/2, getWindowHeight()) / 4, ColorA(1, 1, 1, 1), Font("TAHOMA", 40.0f));
				gl::disableAlphaBlending();
				
			}
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
	displayMode = VIDEO;
	mUserColor = false;
	mPrintGestureData = false;
	mLabel = "";
	mKvalue = 5;
	mDebug = false;
	subTitle = "";


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
	mParams->addParam("Depth", &mEnabledDepth);
	mParams->addParam("Skeletons", &mEnabledSkeletons, "key=k");
	mParams->addParam("Color", &mEnabledColor, "key=v");
	mParams->addParam("Debug Mode", &mDebug, "key=d");
	mParams->addSeparator();
	mParams->addText("INPUT");
	mParams->addParam("Remove background", &mRemoveBackground, "key=b");
	mParams->addParam("Binary depth mode", &mBinaryMode, "key=w");
	mParams->addParam("Invert binary image", &mInverted, "key=i");
	mParams->addParam("Flip input", &mFlipped, "key=m");
	mParams->addParam("Near mode", &mEnabledNearMode, "key=n");
	mParams->addParam("Seated mode", &mEnabledSeatedMode, "key=e");
	mParams->addParam("User color", &mUserColor, "key=u");
	mParams->addParam("K value", &mKvalue);
	mParams->addParam("Print gesture data", &mPrintGestureData, "key=p");
	mParams->addParam("Gesture label", &mLabel, "");
	mParams->addSeparator();
	mParams->addText("APPLICATION");
	mParams->addParam("Full screen", &mFullScreen, "key=f");
	mParams->addButton("Screen shot", bind(&cv_testApp::screenShot, this), "key=s");
	mParams->addButton("Quit", bind(&cv_testApp::quit, this), "key=q");
	

	// initialize gestureData
	ifstream dataFile("train.csv");
	
	string line;
	char *ptr;
	int pos = 0;
	// Init
	huMin = vector<double>(7,999);
	huMax = vector<double>(7,-999);
	
	while (getline(dataFile, line)){
		vector<string> data;
		while ((pos = line.find(",")) != std::string::npos){
			string token = line.substr(0, pos);
			data.push_back(token);
			line.erase(0, pos + 1);
		}
		data.push_back(line);

		vector<double> huMoments(7);
		for (int i = 0; i < 7; i++)
			huMoments[i] = std::stod(data[i+2]);
		Gesture gesture(huMoments, data[0]);
		if (mGestures.size() <= (stoi(data[1])))
			mGestures.resize(stoi(data[1]) + 1);
		mGestures[stoi(data[1])].push_back(gesture);
	}

	for (int i = 0; i < mGestures.size(); i++)
	{		
		for (int j = 0; j < mGestures[i].size(); j++)
		{
			for (int k = 0; k < 7; k++){
				huMin[k] = fmin(huMin[k], mGestures[i][j].hu[k]);
				huMax[k] = fmax(huMax[k], mGestures[i][j].hu[k]);
			}
		}
	}	

	dataFile.close();
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
	mKinect->enableUserColor(mUserColor);
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

	// Toggle User Color
	if (mUserColor != mUserColorPrev) {
		mKinect->enableUserColor(mUserColor);
		mUserColorPrev = mUserColor;
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
	if (mDepthSurface) {
		img_depth = toOcv(Channel8u(mDepthSurface));
	}

	if (!mSkeletons.empty()){
		vector<Skeleton>::iterator skeletonIt = mSkeletons.begin();
		while (skeletonIt->empty()){
			skeletonIt++;
			if (skeletonIt == mSkeletons.end()){
				subTitle = "";
				gestureHistory.clear();
				return;
			}
		}
		vector<JointName> jointList;

		// Get left and righthands
		if (!skeletonIt->empty()){
			const Bone& righthand = skeletonIt->at(NUI_SKELETON_POSITION_HAND_RIGHT);
			Vec2f handPos2d = mKinect->getSkeletonDepthPos(righthand.getPosition());
			float handDepth = righthand.getPosition().z;

			const Bone& lefthand = skeletonIt->at(NUI_SKELETON_POSITION_HAND_LEFT);
			Vec2f lefthandPos2d = mKinect->getSkeletonDepthPos(lefthand.getPosition());
			float lefthandDepth = lefthand.getPosition().z;

			// 2hands together to end
			if (lefthandPos2d.distance(handPos2d) < 50){
				subTitle = "";
				gestureHistory.clear();
				return;
			}

			// hand pos with depth
			Vec3f handPos3d(handPos2d, handDepth);
				
			// get hands from depth surface
			float radius = (1 / handDepth) * 40;
			if (mDepthSurface){
				if (handPos2d.x > radius
					&& handPos2d.x < 320 - radius
					&& handPos2d.y > radius
					&& handPos2d.y < 240 - radius)
				{
					// Hand thresholding

					int rectTopLeftX = handPos2d.x - radius;
					int rectTopLeftY = handPos2d.y - radius;

					int hand_intensity = img_depth.at<uchar>(handPos2d.y, handPos2d.x);

					cv::Rect handRect(rectTopLeftX, rectTopLeftY, radius * 2.0f, radius * 2.0f);
					cv::Mat handMat = img_depth(handRect);

					cv::Mat img_thresh, img_canny;
					cv::threshold(handMat, img_thresh, hand_intensity + 3, 255, CV_THRESH_BINARY_INV);
					cv::medianBlur(img_thresh, img_thresh, 3);
					cv::GaussianBlur(img_thresh, img_thresh, cv::Size(3, 3), 5.0);

					// Find contour
					Canny(img_thresh, img_canny, 60, 110);

					vector< vector<cv::Point> > contours;
					cv::findContours(img_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

					// Filter contours (get only 1 contour)
					if (contours.size() > 1){
						int maxSize = 0;
						int maxIndex = 0;
						for (int i = 0; i < contours.size(); i++){
							if (contours[i].size() > maxSize){
								maxIndex = i;
								maxSize = contours[i].size();
							}
						}
						vector< vector<cv::Point> > tempContours;
						tempContours.push_back(contours[maxIndex]);
						contours = tempContours;
					}

					vector< vector<cv::Point> > hull(contours.size());
					vector< vector<int> > hullI(contours.size());

					vector< vector<cv::Vec4i> > defects(contours.size());

					for (size_t i = 0; i < contours.size(); i++)
					{
						cv::convexHull(contours[i], hull[i], false);
						cv::convexHull(contours[i], hullI[i], false);
						if (contours[i].size() > 3){
							cv::convexityDefects(contours[i], hullI[i], defects[i]);
						}
					}

					vector<double> hu(7);
					// Find hu moments
					if (contours.size() == 1){
						cv::Moments mu;
						mu = moments(contours[0], false);
						cv::HuMoments(mu, hu);
					}

					//draw Contours
					cv::Mat drawing = cv::Mat::zeros(img_thresh.size(), CV_8UC3);
					circle(drawing, cv::Point(radius, radius), 1, cv::Scalar(0, 0, 255), 2);
					for (size_t i = 0; i < contours.size(); i++)
					{
						cv::Scalar color = cv::Scalar(255, 255, 255);
						cv::drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, cv::Point());
						cv::drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, cv::Point());
					}
						
					int defectCount = 0;
					/// Draw convexityDefects
					for (int i = 0; i < contours.size(); ++i)
					{	
						for (const cv::Vec4i& v : defects[i])
						{
							float depth = v[3] / 256;
							if (depth > 10) //  filter defects by depth, e.g more than 10
							{
								int startidx = v[0]; cv::Point ptStart(contours[i][startidx]);
								int endidx = v[1]; cv::Point ptEnd(contours[i][endidx]);
								int faridx = v[2]; cv::Point ptFar(contours[i][faridx]);

								line(drawing, ptStart, ptEnd, cv::Scalar(0, 255, 0), 1);
								line(drawing, ptStart, ptFar, cv::Scalar(0, 255, 0), 1);
								line(drawing, ptEnd, ptFar, cv::Scalar(0, 255, 0), 1);
								circle(drawing, ptFar, 4, cv::Scalar(0, 255, 0), 2);
								defectCount++;
							}
						}
					}

					if (mPrintGestureData){
						console() << mLabel << "," << defectCount << ",";
						for (int i = 0; i < 7; i++){
							console() << hu[i] << ",";
						}
						console() << endl;
						mPrintGestureData = false;
					}
							
					mHandsMat.push_back(drawing);
					mHandPos.push_back(handPos3d);

					string result = kNN(defectCount, Gesture(hu, "unknown"), mKvalue);

					gestureHistory.push_back(result);
					if (gestureHistory.size() > 60){
						gestureHistory.erase(gestureHistory.begin());
					}
					else{
						return;
					}

					// find history mode
					map<string, int> count;
					for (int i = 0; i < gestureHistory.size(); i++){
						count[gestureHistory[i]]++;
					}

					int maxCount = -1;
					string maxName;
					for (map<string, int>::iterator it = count.begin(); it != count.end(); it++){
						if (it->second > maxCount){
							maxCount = it->second;
							maxName = it->first;
						}
					}

					cv::putText(drawing, maxName, cv::Point(0, radius * 2), CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255));
					if (subTitle == ""){
						subTitle = maxName;
					}
					else if (lastSubTitle != maxName) {
						subTitle += " " + maxName;
					}
					lastSubTitle = maxName;

					//console() << "Recognized Gesture: " <<  kNN(defectCount, Gesture(hu, "unknown")) << endl;
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

string cv_testApp::kNN(int defect, Gesture gesture,int k){
	if (defect >= mGestures.size())
		return "";

	vector<Gesture> trainingData = mGestures[defect];
	vector<pair<double, string> > difference;

	// find difference with every attributes
	for (int i = 0; i < trainingData.size(); i++){
		double sum = 0;
		for (int j = 0; j < 7; j++){
			double diff = gesture.hu[j] - trainingData[i].hu[j];
			diff /= (huMax[j] - huMin[j]);
			sum += abs(diff);
		}
		difference.push_back( make_pair(sum, trainingData[i].name) );
	}

	// sort
	sort(difference.begin(), difference.end());

	//thresholding
	//console() << difference[k - 1].first << endl;
	if (difference[k - 1].first > 0.9){
		return "";
	}

	// find k nearest neighbor
	map<string, int> count;
	for (int i = 0; i < k; i++){
		count[difference[i].second]++;
	}

	

	int maxCount = -1;
	string maxName;
	for (map<string, int>::iterator it = count.begin(); it != count.end(); it++){
		if (it->second > maxCount){
			maxCount = it->second;
			maxName = it->first;
		}
	}

	return maxName;
}

// Run application
CINDER_APP_BASIC(cv_testApp, RendererGl)
