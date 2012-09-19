
#include <cstdio>
#include <ctime>
#include <csignal>

#include <XnCppWrapper.h>
#include <XnUSB.h>

#include <iostream>
#include <string>

#include <math.h>

using namespace std;

class KinectMotors
{
public:
    enum { MaxDevs = 16 };
  enum LED_STATUS {
    LED_OFF = 0,
    LED_GREEN,
    LED_RED,
    LED_ORANGE,
    LED_BLINK_ORANGE,
    LED_BLINK_GREEN,
    LED_BLINK_RED_ORANGE
  };
  enum STATUS {
    STOPED = 0,
    AT_LIMIT,
    MOVING = 4,
    QUICK_BREAK = 8,
    UNKNOW = -1
  };

  class Device
  {
  private:
    XN_USB_DEV_HANDLE handle;
    bool m_isOpen;

    Device(const Device& d);
    Device& operator =(const Device& d);

  public:
    Device() : handle(NULL), m_isOpen(false) {}
    Device(XN_USB_DEV_HANDLE h) : handle(h), m_isOpen(false) {}
    virtual ~Device() { if (m_isOpen) Close(); }

    void SetHandle(XN_USB_DEV_HANDLE h) { handle = h; }

    /**
     * Open device.
     * @return true if succeeded, false - overwise
     */
    bool Open();

    /**
     * Close device.
     */
    void Close();

    /**
     * Move motor up or down to specified angle value.
     * @param angle angle value
     * @return true if succeeded, false - overwise
     */
    bool Move(int angle);

    /**
     * Set Led Status
     * @param status status code
     * @return true if succeeded, false - overwise
     */
    bool SetLed(int status);

    /**
     * Get current status (status code, speed, angle)
     * @param status motor status code
     * @param speed current speed if in moving
     * @param angle current angle value
     * @return true if succeeded, false - overwise
     */
    bool GetStatus(int& status, int& speed, int& angle);
  };

    KinectMotors();
    virtual ~KinectMotors();

  bool Initialize();
  size_t Count() { return m_num; }
  Device& operator [](const int idx) {
    return m_devs[idx];
  }

private:
  Device m_devs[MaxDevs];
    XnUInt32 m_num;
};

KinectMotors::KinectMotors() : m_num(0)
{
}

KinectMotors::~KinectMotors()
{
}

bool KinectMotors::Initialize()
{
    const XnUSBConnectionString *paths;
    XnUInt32 count;
    XnStatus res;

    // Init OpenNI USB
    res = xnUSBInit();
    if (res != XN_STATUS_OK)
    {
        xnPrintError(res, "xnUSBInit failed");
        return false;
    }

    // Open all "Kinect motor" USB devices
    res = xnUSBEnumerateDevices(0x045E /* VendorID */, 0x02B0 /*ProductID*/, &paths, &count);
    if (res != XN_STATUS_OK)
    {
        xnPrintError(res, "xnUSBEnumerateDevices failed");
        return false;
    }

    // Open devices
  XN_USB_DEV_HANDLE h;
    for (XnUInt32 index = 0; index < count; ++index)
    {
        res = xnUSBOpenDeviceByPath(paths[index], &h);
        if (res != XN_STATUS_OK) {
            xnPrintError(res, "xnUSBOpenDeviceByPath failed");
            return false;
        }
    m_devs[index].SetHandle(h);
    }

  m_num = count;
  return true;
}

bool KinectMotors::Device::Open()
{
  if (handle == NULL) return false;

    XnStatus res;
    XnUChar buf[1]; // output buffer

    // Init motors
  res = xnUSBSendControl(handle, (XnUSBControlType) 0xc0, 0x10, 0x00, 0x00, buf, sizeof(buf), 0);
  if (res != XN_STATUS_OK) {
    xnPrintError(res, "xnUSBSendControl failed");
    Close();
    return false;
  }

  res = xnUSBSendControl(handle, XN_USB_CONTROL_TYPE_VENDOR, 0x06, 0x01, 0x00, NULL, 0, 0);
  if (res != XN_STATUS_OK) {
    xnPrintError(res, "xnUSBSendControl failed");
    Close();
    return false;
  }

    m_isOpen = true;

    return true;
}

void KinectMotors::Device::Close()
{
    if (m_isOpen) {
    xnUSBCloseDevice(handle);
    m_isOpen = false;
    }
}

bool KinectMotors::Device::Move(int angle)
{
    XnStatus res;

    // Send move control requests
  res = xnUSBSendControl(handle, XN_USB_CONTROL_TYPE_VENDOR, 0x31, 2*angle, 0x00, NULL, 0, 0);

  if (res != XN_STATUS_OK)
  {
    xnPrintError(res, "xnUSBSendControl failed");
    return false;
  }
    return true;
}

bool KinectMotors::Device::SetLed(int status)
{
    XnStatus res;
  res = xnUSBSendControl(handle, XN_USB_CONTROL_TYPE_VENDOR, 0x06, status, 0x00, NULL, 0, 0);

  if (res != XN_STATUS_OK)
  {
    xnPrintError(res, "xnUSBSendControl failed");
    return false;
  }
    return true;
}

bool KinectMotors::Device::GetStatus(int& status, int& speed, int& angle)
{
    XnStatus res;
  XnUChar buf[10] = {0};
  XnUInt32 size = 0;

  res = xnUSBReceiveControl(handle, XN_USB_CONTROL_TYPE_VENDOR, 0x32, 0, 0, buf, 10, &size, 0);

  if (res != XN_STATUS_OK)
  {
    xnPrintError(res, "xnUSBSendControl failed");
    return false;
  }

  status = static_cast<int>(buf[9]);
  speed = static_cast<int>(buf[1]);
  angle = static_cast<int>(static_cast<char>(buf[8]))/2;

    return true;
}


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

KinectMotors motors;

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
	printf("{\"lost_user\":{\"userid\":%d}}\n", nId);
	userGenerator.GetSkeletonCap().StopTracking(nId);
	userGenerator.GetSkeletonCap().Reset(nId);
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

	if (motors.Count() > 0) {
		KinectMotors::Device& dev = motors[0];
		dev.SetLed(KinectMotors::LED_RED);
	}
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

	if (motors.Count() > 0) {
		KinectMotors::Device& dev = motors[0];
		dev.SetLed(KinectMotors::LED_GREEN);
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

	int skeletons = 0;
	int activeUsers = 0;
  int totalJoints;
  string userSkeleton;
  string activeSkeletons[nUsers];

  userGenerator.GetUsers(aUsers, nUsers);
  for (int i = 0; i < nUsers; ++i) {
    totalJoints = 0;
    userSkeleton = "";

    char tmp[1024];
    sprintf(tmp, "{\"userid\":%d,\"joints\":[", aUsers[i]);
    userSkeleton += tmp;

    if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i])) {
			if (jointPos(aUsers[i], XN_SKEL_HEAD) == 0) {
        writeJoint(&userSkeleton, "head", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_NECK) == 0) {
				writeJoint(&userSkeleton, "neck", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_COLLAR) == 0) {
				writeJoint(&userSkeleton, "l_collar", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_SHOULDER) == 0) {
				writeJoint(&userSkeleton, "l_shoulder", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_ELBOW) == 0) {
				writeJoint(&userSkeleton, "l_elbow", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_WRIST) == 0) {
				writeJoint(&userSkeleton, "l_wrist", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_HAND) == 0) {
				writeJoint(&userSkeleton, "l_hand", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_FINGERTIP) == 0) {
				writeJoint(&userSkeleton, "l_fingertop", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_COLLAR) == 0) {
				writeJoint(&userSkeleton, "r_collar", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_SHOULDER) == 0) {
				writeJoint(&userSkeleton, "r_shoulder", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_ELBOW) == 0) {
				writeJoint(&userSkeleton, "r_elbow", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_WRIST) == 0) {
				writeJoint(&userSkeleton, "r_wrist", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_HAND) == 0) {
				writeJoint(&userSkeleton, "r_hand", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_FINGERTIP) == 0) {
				writeJoint(&userSkeleton, "r_fingertip", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_TORSO) == 0) {
				writeJoint(&userSkeleton, "torso", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_WAIST) == 0) {
				writeJoint(&userSkeleton, "waist", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_HIP) == 0) {
				writeJoint(&userSkeleton, "l_hip", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_KNEE) == 0) {
				writeJoint(&userSkeleton, "l_knee", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_ANKLE) == 0) {
				writeJoint(&userSkeleton, "l_ankle", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_LEFT_FOOT) == 0) {
				writeJoint(&userSkeleton, "l_foot", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_HIP) == 0) {
				writeJoint(&userSkeleton, "r_hip", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_KNEE) == 0) {
				writeJoint(&userSkeleton, "r_knee", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_ANKLE) == 0) {
				writeJoint(&userSkeleton, "r_ankle", jointCoords);
        totalJoints++;
			}
			if (jointPos(aUsers[i], XN_SKEL_RIGHT_FOOT) == 0) {
				writeJoint(&userSkeleton, "r_foot", jointCoords);
        totalJoints++;
			}

      if(totalJoints > 0){
        userSkeleton.erase(userSkeleton.length()-1, 1);
      }
		}
		userSkeleton += "]}";

    if(totalJoints > 0){
      activeSkeletons[i] = userSkeleton;
      skeletons++;
    }
	}

  string concatSkeletons;
  concatSkeletons = "";
  for(int ai = 0; ai < skeletons; ++ai)
  {
    if(ai != 0){
      concatSkeletons += ",";
    }

    concatSkeletons += activeSkeletons[ai];
  }

  s += concatSkeletons;

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
	motors.Initialize();

  if (motors.Count() > 0) {
    KinectMotors::Device& dev = motors[0];
    dev.SetLed(KinectMotors::LED_ORANGE);
  }

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