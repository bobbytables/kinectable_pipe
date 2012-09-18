
#include <cstdio>
#include <ctime>
#include <csignal>

#include <XnCppWrapper.h>

#include <iostream>
#include <string>

#include <math.h>

using namespace std;

bool saveRgbImage = false;
bool saveDepthImage = false;


int userID;
float jointCoords[3];
float jointOrients[9];

float posConfidence;
float orientConfidence;

// hand data
float handCoords[3];
bool haveHand = false;

bool sendRot = false;
int nDimensions = 3;

xn::Context context;

xn::DepthGenerator depth;
xn::DepthMetaData depthMD;
xn::ImageMetaData imageMD;
xn::AudioMetaData audioMD;

xn::ImageGenerator image;
xn::UserGenerator userGenerator;
xn::HandsGenerator handsGenerator;
xn::GestureGenerator gestureGenerator;
xn::AudioGenerator audioGenerator;

XnChar g_strPose[20] = "";
#define GESTURE_TO_USE "Wave"

// framerate related config
double FRAMERATE = 30;
std::clock_t last;

float clockAsFloat(std::clock_t t) {
	return t / (double) CLOCKS_PER_SEC;
}

//gesture callbacks
void XN_CALLBACK_TYPE Gesture_Recognized(xn::GestureGenerator& generator, const XnChar* strGesture, const XnPoint3D* pIDPosition, const XnPoint3D* pEndPosition, void* pCookie) {
	printf("{\"gesture\":{\"type\":\"%s\"}, \"elapsed\":%.3f}}\n", strGesture, clockAsFloat(last));
	gestureGenerator.RemoveGesture(strGesture);
	handsGenerator.StartTracking(*pEndPosition);
}

void XN_CALLBACK_TYPE Gesture_Process(xn::GestureGenerator& generator, const XnChar* strGesture, const XnPoint3D* pPosition, XnFloat fProgress, void* pCookie) {
}

//hand callbacks new_hand, update_hand, lost_hand
void XN_CALLBACK_TYPE new_hand(xn::HandsGenerator &generator, XnUserID nId, const XnPoint3D *pPosition, XnFloat fTime, void *pCookie) {
//	printf("{'found_hand\":{\"userid\":%d,'x':%.3f,'y':%.3f,'z':%.3f}}\n", nId, pPosition->X, pPosition->Y, pPosition->Z);
}
void XN_CALLBACK_TYPE lost_hand(xn::HandsGenerator &generator, XnUserID nId, XnFloat fTime, void *pCookie) {
	printf("{\"lost_hand\":{\"userid\":%d}, \"elapsed\":%.3f}}\n", nId, clockAsFloat(last));
	gestureGenerator.AddGesture(GESTURE_TO_USE, NULL);
}

void XN_CALLBACK_TYPE update_hand(xn::HandsGenerator &generator, XnUserID nId, const XnPoint3D *pPosition, XnFloat fTime, void *pCookie) {
//	printf("{'update_hand\":{\"userid\":%d,'x':%.3f,'y':%.3f,'z':%.3f}}\n", nId, pPosition->X, pPosition->Y, pPosition->Z);
	haveHand = true;
	handCoords[0] = pPosition->X;
	handCoords[1] = pPosition->Y;
	handCoords[2] = pPosition->Z;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE new_user(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("{\"found_user\":{\"userid\":%d}, \"elapsed\":%.3f}\n", nId, clockAsFloat(last));
	userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}



// Callback: An existing user was lost
void XN_CALLBACK_TYPE lost_user(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	// printf("{\"lost_user\":{\"userid\":%d}}\n", nId);
	// userGenerator.GetSkeletonCap().StopTracking(nId);
	// userGenerator.GetSkeletonCap().Reset(nId);
}

void XN_CALLBACK_TYPE user_exit(xn::UserGenerator& generator, XnUserID nId, void *pCookie) {
	printf("{\"user_exit\": {\"userid\": %d}}\n", nId);
	userGenerator.GetSkeletonCap().StopTracking(nId);
	userGenerator.GetSkeletonCap().Reset(nId);
}

void XN_CALLBACK_TYPE user_reenter(xn::UserGenerator& generator, XnUserID nId, void *pCookie) {
	printf("{\"user_reenter\": {\"userid\": %d}}\n", nId);
	userGenerator.GetSkeletonCap().StartTracking(nId);
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE pose_detected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie) {
//	printf("{'pose_detected\":{\"userid\":%d,'type':'%s'}\n", nId, strPose);
	userGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}



// Callback: Started calibration
void XN_CALLBACK_TYPE calibration_started(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	last = std::clock();
	printf("{\"calibration_started\":{\"userid\":%d}, \"elapsed\":%.3f}\n", nId, clockAsFloat(last));
}



// Callback: Finished calibration
void XN_CALLBACK_TYPE calibration_ended(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		printf("{\"calibration_ended\":{\"userid\":%d}, \"elapsed\":%.3f}\n", nId, clockAsFloat(last));
		userGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		printf("{\"calibration_failed\":{\"userid\":%d}, \"elapsed\":%.3f}\n", nId, clockAsFloat(last));
		userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

int jointPos(XnUserID player, XnSkeletonJoint eJoint) {

	XnSkeletonJointTransformation jointTrans;

	userGenerator.GetSkeletonCap().GetSkeletonJoint(player, eJoint, jointTrans);

	posConfidence = jointTrans.position.fConfidence;

	userID = player;

	jointCoords[0] = jointTrans.position.position.X;
	jointCoords[1] = jointTrans.position.position.Y;
	jointCoords[2] = jointTrans.position.position.Z;

	orientConfidence = jointTrans.orientation.fConfidence;

	for (int i=0; i<9; i++)
	{
		jointOrients[i] = jointTrans.orientation.orientation.elements[i];
	}

	return 0;
}

void writeUserPosition(string *s, XnUserID id) {
	XnPoint3D com;
	userGenerator.GetCoM(id, com);

	if (fabsf( com.X - 0.0f ) > 0.1f)
	{
		char tmp[1024];

		sprintf(tmp, "{\"userid\":%u,\"X\":%.3f,\"Y\":%.3f,\"Z\":%.3f}\n", id, com.X, com.Y, com.Z);
		*s += tmp;
	}
}

void writeHand() {
	if (!haveHand)
		return;

	jointCoords[0] = handCoords[0];
	jointCoords[1] = handCoords[1];
	jointCoords[2] = handCoords[2];

//	printf("{'hand':{'X':%.3f,'Y':%.3f,'Z':%.3f}},", handCoords[0], handCoords[1], handCoords[2]);
	haveHand = false;
}

bool validJoint(float* jointCoords) {

	for (int i=0; i < 3; i++)
	{
		if (jointCoords[i] == 0.0f) return false;
		if (fabsf( jointCoords[i] - 0.0f ) < 0.01f) return false;
		if (jointCoords[i] > 100000.0f) return false;
		if (jointCoords[i] < -100000.0f) return false;
	}

	return true;
}

void writeJoint(string *s, char* t, float* jointCoords) {
	char tmp[1024];
	if (validJoint(jointCoords))
	{
		sprintf(tmp, "{\"joint\":\"%s\",\"X\":%.3f,\"Y\":%.3f,\"Z\":%.3f},", t, jointCoords[0], jointCoords[1], jointCoords[2]);
	}
	*s += tmp;
}

void writeSkeleton() {
	string s;

	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;

	s += "{\"skeletons\":[";

	int skeletons=0;

	userGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i) {
		if (!userGenerator.GetSkeletonCap().IsTracking(aUsers[i])) {
			continue;
		}

		if (i > 0)
		{
			s += ",";
		}
		char tmp[1024];
		sprintf(tmp, "{\"userid\":%d,\"joints\":[", i);
		s += tmp;

		if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i])) {
			skeletons++;
			if (jointPos(aUsers[i], XN_SKEL_HEAD) == 0) {
				writeJoint(&s, "head", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_NECK) == 0) {
				writeJoint(&s, "neck", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_COLLAR) == 0) {
				writeJoint(&s, "l_collar", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_SHOULDER) == 0) {
				writeJoint(&s, "l_shoulder", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_ELBOW) == 0) {
				writeJoint(&s, "l_elbow", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_WRIST) == 0) {
				writeJoint(&s, "l_wrist", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_HAND) == 0) {
				writeJoint(&s, "l_hand", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_FINGERTIP) == 0) {
				writeJoint(&s, "l_fingertop", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_COLLAR) == 0) {
				writeJoint(&s, "r_collar", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_SHOULDER) == 0) {
				writeJoint(&s, "r_shoulder", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_ELBOW) == 0) {
				writeJoint(&s, "r_elbow", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_WRIST) == 0) {
				writeJoint(&s, "r_wrist", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_HAND) == 0) {
				writeJoint(&s, "r_hand", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_FINGERTIP) == 0) {
				writeJoint(&s, "r_fingertip", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_TORSO) == 0) {
				writeJoint(&s, "torso", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_WAIST) == 0) {
				writeJoint(&s, "waist", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_HIP) == 0) {
				writeJoint(&s, "l_hip", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_KNEE) == 0) {
				writeJoint(&s, "l_knee", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_ANKLE) == 0) {
				writeJoint(&s, "l_ankle", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_FOOT) == 0) {
				writeJoint(&s, "l_foot", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_HIP) == 0) {
				writeJoint(&s, "r_hip", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_KNEE) == 0) {
				writeJoint(&s, "r_knee", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_ANKLE) == 0) {
				writeJoint(&s, "r_ankle", jointCoords);
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_FOOT) == 0) {
				writeJoint(&s, "r_foot", jointCoords);
			}
			s.erase(s.length()-1, 1);
		}
		else {
			//Send user's center of mass
			writeUserPosition(&s, aUsers[i]);
		}
		s += "]}";
	}
	// add a timestamp
	char tmp[1024];
	sprintf(tmp, "],\"elapsed\":%.3f}", clockAsFloat(last));
	s += tmp;

	if (skeletons > 0)
	{
		cout << s;
		cout << endl;
		cout.flush();
	}
	else
	{
		s.clear();
	}
	skeletons=0;
}

int usage(char *name) {
	printf("\nUsage: %s [OPTIONS]\n\
		Example: %s -r 30\n\
		\n\
		(The above example corresponds to the defaults)\n\
		\n\
		Options:\n\
		-r <n>\t framerate\n\
		-i\t save rgb image\n\
		-d\t save depth image\n\
		For a more detailed explanation of options consult the README file.\n\n",
		name, name);
	exit(1);
}

void checkRetVal(XnStatus nRetVal) {
	if (nRetVal != XN_STATUS_OK) {
		printf("There was a problem initializing kinect... Make sure you have \
			connected both usb and power cables and that the driver and OpenNI framework \
			are correctly installed.\n\n");
		exit(1);
	}
}

void terminate(int ignored) {
	context.Shutdown();
	exit(0);
}

// code adapted from https://groups.google.com/group/openni-dev/tree/browse_frm/month/2011-03/c40f876672bb714c?rnum=11&lnk=nl
#define MAX_DEPTH 10000

void main_loop() {
	// Read next available data
	context.WaitAnyUpdateAll();

	// Process the images
	depth.GetMetaData(depthMD);

	image.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
	image.GetMetaData(imageMD);

	writeSkeleton();

	fflush(stdout);
}

int main(int argc, char **argv) {
	last = std::clock();

	printf("{\"status\":\"initializing\", \"elapsed\":%0.3f}\n", clockAsFloat(last));
	fflush(stdout);

	unsigned int arg = 1,
		require_argument = 0,
		port_argument = 0;
	XnMapOutputMode mapMode;
	XnStatus nRetVal = XN_STATUS_OK;
	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks, hHandsCallbacks;
	xn::Recorder recorder;

	context.Init();

	checkRetVal(depth.Create(context));
	checkRetVal(image.Create(context));

	nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
	if (nRetVal != XN_STATUS_OK)
		nRetVal = userGenerator.Create(context);

	checkRetVal(userGenerator.RegisterUserCallbacks(new_user, lost_user, NULL, hUserCallbacks));
	checkRetVal(userGenerator.RegisterToUserExit(user_exit, NULL, hUserCallbacks));
	checkRetVal(userGenerator.RegisterToUserReEnter(user_reenter, NULL, hUserCallbacks));

	checkRetVal(userGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(calibration_started, calibration_ended, NULL, hCalibrationCallbacks));
	checkRetVal(userGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(pose_detected, NULL, NULL, hPoseCallbacks));
	checkRetVal(userGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose));
	checkRetVal(userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL));
	userGenerator.GetSkeletonCap().SetSmoothing(0.8);

	signal(SIGTERM, terminate);
	signal(SIGINT, terminate);

	printf("{\"status\":\"seeking_users\", \"elapsed\":%.3f}\n", clockAsFloat(last));
	fflush(stdout);
	context.StartGeneratingAll();

	while(true)
		main_loop();

	terminate(0);
}