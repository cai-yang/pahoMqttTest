#include<iostream>
#include<string>
#include<vector>
#include<math.h>
#include<Windows.h>
#include "mqtt\async_client.h"
#include<signal.h>
#define PI 3.1415927

using namespace std;

bool flag = false;
float offset[5] = {};
mqtt::string rightHandTopic[5] = { "gloves/r1","gloves/r2","gloves/r3","gloves/r4","gloves/r5" };
mqtt::string leftHandTopic[5] = { "gloves/l1","gloves/l2","gloves/l3","gloves/l4","gloves/l5" };
mqtt::string rightHandRawTopic[36] = {};
mqtt::string leftHandRawTopic[36] = {};
string quadTextList[4] = { "w","x","y","z" };


enum SpringVR_eModel {
	//M系列有手掌和手指
	M7 = 0x07,
	M8 = 0x08,
	M9 = 0x09,
	//S系列有前臂、手掌和手指
	S8 = 0x18,
	//Z系列有上臂、前臂、手掌和手指
	Z8 = 0x28,
	Z9 = 0x29,
	//B系列包括全身（下身、上身、右手、左手）
	B8 = 0xB8
};
enum SpringVR_ePart {
	eRighthand = 0x80,
	eLefthand,
	eUpperbody,
	eLowerbody
};
enum SpringVR_eCmd {
	eCommon = 0x00,
	eClick = 0x02,
	eHold,
	eRelease,
	eStaticCalibrationEnd,
	eDynamicCalibrationEnd
};
struct SpringVR_Quat {
	float w;
	float x;
	float y;
	float z;
};

struct EulerAngle {
	float heading;
	float attitude;
	float bank;
};

EulerAngle Quaternion2Euler(SpringVR_Quat q) {
	EulerAngle out;
	out.heading = atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * pow(q.y, 2) - 2 * pow(q.z, 2))/PI*180;
	out.attitude = asin(2 * q.x * q.y + 2 * q.z * q.w)/PI*180;
	out.bank = atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * pow(q.x, 2) - 2 * pow(q.z, 2))/PI*180;
	return out;	
};

struct SpringVR_Data {
	BOOL bReceivedOneFrameData;
	SpringVR_eCmd eCmd;
	vector<SpringVR_Quat> vQuat;
};

SpringVR_Quat Mul_Quat(SpringVR_Quat a, SpringVR_Quat b) {
	SpringVR_Quat q;
	q.w = (a.w*b.w - a.x * b.x - a.y * b.y - a.z * b.z);
	q.x = (a.w*b.x + a.x * b.w + a.y * b.z - a.z * b.y);
	q.y = (a.w*b.y - a.x * b.z + a.y * b.w + a.z * b.x);
	q.z = (a.w*b.z + a.x * b.y - a.y * b.x + a.z * b.w);
	return q;
};

float Mod_Quat(SpringVR_Quat a) {
	return sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);
};

SpringVR_Quat Normalize_Quat(SpringVR_Quat q) {
	float m = Mod_Quat(q);
	SpringVR_Quat out;
	out.w = q.w / m;
	out.x = q.x / m;
	out.y = q.y / m;
	out.z = q.z / m;
	return out;
};

SpringVR_Quat Rev_Quat(SpringVR_Quat a) {
	SpringVR_Quat q;
	q.w = a.w;
	q.x = -a.x;
	q.y = -a.y;
	q.z = -a.z;
	return q;
};

UINT8 SpringVR__StaticCalibration__Cmd[7] = { 0xFE, 0x80, 0x02, 0x00, 0x00, 0x82, 0xFD };
UINT8 SpringVR__DynamicCalibration__Cmd[7] = { 0xFE, 0x80, 0x04, 0x00, 0x00, 0x84, 0xFD };
SpringVR_eModel SpringVR_Model = SpringVR_eModel::M7;
SpringVR_Data SpringVR_Righthand;
SpringVR_Data SpringVR_Lefthand;
SpringVR_Data SpringVR_Upperbody;
SpringVR_Data SpringVR_Lowerbody;

typedef void(*SpringVR_pcbComm_GetData)(UINT8 part_i, UINT8 cmd_i, float *pData_i, INT32 dataNum_i);
typedef void(*SpringVR_pcbComm_Closed)();
typedef BOOL(*SpringVR_pComm_Init)(UINT8 model_i, SpringVR_pcbComm_GetData cb_i);
typedef BOOL(*SpringVR_pComm_WriteData)(UINT8 part_i, UINT8 *data_i, UINT8 dataNum_i);
typedef BOOL(*SpringVR_pComm_Close)(SpringVR_pcbComm_Closed cb_i);

SpringVR_pComm_Init SpringVR_Comm_Init;
SpringVR_pComm_WriteData SpringVR_Comm_WriteData;
SpringVR_pComm_Close SpringVR_Comm_Close;

void SpringVR_cbComm_GetData(UINT8 part_i, UINT8 cmd_i, float *pData_i, INT32 dataNum_i) {
	/*cout << "which_part = 0x" << hex << +part_i\
	<< "; which_cmd = 0x" << hex << +cmd_i\
	<< "; data_floats = " << dec << dataNum_i\
	<< endl;*/
	INT32 quatNum_a = dataNum_i / 4;
	vector<SpringVR_Quat> vQuat_a;
	for (int i = 0; i < quatNum_a; i++) {
		vQuat_a.push_back(SpringVR_Quat{ pData_i[i * 4 + 0], pData_i[i * 4 + 1], pData_i[i * 4 + 2], pData_i[i * 4 + 3] });
	}
	switch ((SpringVR_ePart)part_i) {
	case(SpringVR_ePart::eRighthand):
	{
		SpringVR_Righthand.eCmd = (SpringVR_eCmd)cmd_i;
		SpringVR_Righthand.vQuat = vQuat_a;
		SpringVR_Righthand.bReceivedOneFrameData = true;
		break;
	}
	case(SpringVR_ePart::eLefthand):
	{
		SpringVR_Lefthand.eCmd = (SpringVR_eCmd)cmd_i;
		SpringVR_Lefthand.vQuat = vQuat_a;
		SpringVR_Lefthand.bReceivedOneFrameData = true;
		break;
	}
	case(SpringVR_ePart::eUpperbody):
	{
		SpringVR_Upperbody.eCmd = (SpringVR_eCmd)cmd_i;
		SpringVR_Upperbody.vQuat = vQuat_a;
		SpringVR_Upperbody.bReceivedOneFrameData = true;
		break;
	}
	case(SpringVR_ePart::eLowerbody):
	{
		SpringVR_Lowerbody.eCmd = (SpringVR_eCmd)cmd_i;
		SpringVR_Lowerbody.vQuat = vQuat_a;
		SpringVR_Lowerbody.bReceivedOneFrameData = true;
		break;
	}
	default:
	{
		break;
	}
	}
}
void SpringVR_cbComm_Closed() {
	cout << "调用了关闭函数" << endl;
}

int main(int argc, char* argv[])

{

	int count = 0;
	for (int i = 0; i < 36; i++) {
		leftHandRawTopic[i] = "gloves/raw/left/" + to_string(int(count / 4 + 1)) + "/" + quadTextList[count % 4];
		rightHandRawTopic[i] = "gloves/raw/right/" + to_string(int(count / 4 + 1)) + "/" + quadTextList[count % 4];
		count++;
	}

	const std::string CLIENT_ID{ "mqtt_gloves_client" };

	const std::string ADDRESS{ "192.168.1.109:1883" };

	const int QOS = 0;

	const char * PAYLOAD = { "Hello World!" };

	// Create a client





	mqtt::async_client cli(ADDRESS, CLIENT_ID);



	mqtt::connect_options connOpts;

	connOpts.set_keep_alive_interval(60);

	connOpts.set_clean_session(true);



	try {

		std::cout << "Connecting to the MQTT server..." << std::flush;

		cli.connect(connOpts)->wait();

		cli.start_consuming();

		// Now try with itemized publish.



		//cli.publish(TOPIC, PAYLOAD, strlen(PAYLOAD), QOS, false);



		// Disconnect

		/*std::cout << "OK" << std::endl;

		std::cout << "\nShutting down and disconnecting from the MQTT server..." << std::flush;

		cli.unsubscribe(TOPIC)->wait();

		cli.stop_consuming();

		cli.disconnect()->wait();

		std::cout << "OK" << std::endl;*/

	}

	catch (const mqtt::exception& exc) {

		std::cerr << exc.what() << std::endl;

		return 1;

	}


	HMODULE pSpringVR_Comm_a = LoadLibrary("SpringVR_Comm.dll");
	if (pSpringVR_Comm_a != nullptr) {
	cout << "SpringVR_Comm.dll加载成功！" << endl;

	//加载SpringVR_Comm_Init()
	SpringVR_Comm_Init = (SpringVR_pComm_Init)GetProcAddress(pSpringVR_Comm_a, TEXT("SpringVR_Comm_Init"));
	if (SpringVR_Comm_Init != nullptr) {
		cout << "SpringVR_Comm.dll：SpringVR_Comm_Init加载成功！" << endl;
		if (SpringVR_Comm_Init(SpringVR_Model, SpringVR_cbComm_GetData)) {//调用初始化函数SpringVR_Comm_Init()
			cout << "SpringVR_Comm.dll：SpringVR_Comm_Init调用成功！" << endl;
		}
	}
	else {
		cout << "SpringVR_Comm.dll：SpringVR_Comm_Init加载失败！" << endl;
	}

	//加载SpringVR_Comm_WriteData()
	SpringVR_Comm_WriteData = (SpringVR_pComm_WriteData)GetProcAddress(pSpringVR_Comm_a, TEXT("SpringVR_Comm_WriteData"));
	if (SpringVR_Comm_WriteData != nullptr) {
		cout << "SpringVR_Comm.dll：SpringVR_Comm_WriteData加载成功！" << endl;
	}
	else {
		cout << "SpringVR_Comm.dll：SpringVR_Comm_WriteData加载失败！" << endl;
	}

	//加载SpringVR_Comm_Close()
	SpringVR_Comm_Close = (SpringVR_pComm_Close)GetProcAddress(pSpringVR_Comm_a, TEXT("SpringVR_Comm_Close"));
	if (SpringVR_Comm_Close != nullptr) {
		cout << "SpringVR_Comm.dll：SpringVR_Comm_Close加载成功！" << endl;
	}
	else {
		cout << "SpringVR_Comm.dll：SpringVR_Comm_Close加载失败！" << endl;
	}
	}
 else {
	 cout << "SpringVR_Comm.dll加载失败！" << endl;
 }
 
 while (pSpringVR_Comm_a != nullptr) {
	 if (SpringVR_Righthand.bReceivedOneFrameData) {
		 SpringVR_Righthand.bReceivedOneFrameData = FALSE;
		 cout << "收到了右手的数据，请处理……" << endl;
		 SpringVR_Quat list[9] = {};
		 for (int i = 0; i < SpringVR_Righthand.vQuat.size(); i++) {
			 list[i] = SpringVR_Righthand.vQuat.at(i);
		 }
		 
		 float payload[6] = {};
		 payload[0] = 180-2*abs(Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]),list[8])).attitude);
		 payload[1] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]),list[4])).heading;
		 payload[2] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]),list[5])).heading;
		 payload[3] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]),list[6])).heading;
		 payload[4] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]),list[7])).heading;
		 for (int k = 0; k <= 4; k++) {
			 cout <<payload[k]-payload[5]<< endl;
			 cli.publish(rightHandTopic[k], to_string(payload[k]/* - payload[5]*/).c_str(), to_string(payload[k]/* - payload[5]*/).size(), QOS, false);
		 }
		/*
		 int count = 0;
		 for (int i = 0; i < SpringVR_Righthand.vQuat.size(); i++) {
			  cli.publish(rightHandRawTopic[count++], to_string(list[i].w).c_str(), to_string(list[i].w).size(), QOS, false);
			  cli.publish(rightHandRawTopic[count++], to_string(list[i].x).c_str(), to_string(list[i].x).size(), QOS, false);
			  cli.publish(rightHandRawTopic[count++], to_string(list[i].y).c_str(), to_string(list[i].y).size(), QOS, false);
			  cli.publish(rightHandRawTopic[count++], to_string(list[i].z).c_str(), to_string(list[i].z).size(), QOS, false);
		 }
		*/


	 }
	 if (SpringVR_Lefthand.bReceivedOneFrameData) {
		 SpringVR_Lefthand.bReceivedOneFrameData = FALSE;
		 cout << "收到了左手的数据，请处理……" << endl;
		 SpringVR_Quat list[9] = {};
		 for (int i = 0; i < SpringVR_Lefthand.vQuat.size(); i++) {
			 list[i] = SpringVR_Lefthand.vQuat.at(i);
		 }
		 mqtt::string topic[5] = { "gloves/l1","gloves/l2","gloves/l3","gloves/l4","gloves/l5" };
		 float payload[6] = {};
		 payload[0] = 180-2*abs(Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]), list[3])).attitude);
		 payload[1] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]), list[4])).heading;
		 payload[2] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]), list[5])).heading;
		 payload[3] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]), list[6])).heading;
		 payload[4] = Quaternion2Euler(Mul_Quat(Rev_Quat(list[2]), list[7])).heading;
		 for (int k = 0; k <= 4; k++) {
			 cout << payload[k] - payload[5] << endl;
			 cli.publish(topic[k], to_string(payload[k]/* - payload[5]*/).c_str(), to_string(payload[k]/* - payload[5]*/).size(), QOS, false);
		 }
		 /*
		 int count = 0;
		 for (int i = 0; i < SpringVR_Lefthand.vQuat.size(); i++) {
			 cli.publish(leftHandRawTopic[count++], to_string(list[i].w).c_str(), to_string(list[i].w).size(), QOS, false);
			 cli.publish(leftHandRawTopic[count++], to_string(list[i].x).c_str(), to_string(list[i].x).size(), QOS, false);
			 cli.publish(leftHandRawTopic[count++], to_string(list[i].y).c_str(), to_string(list[i].y).size(), QOS, false);
			 cli.publish(leftHandRawTopic[count++], to_string(list[i].z).c_str(), to_string(list[i].z).size(), QOS, false);
		 }
		 */

	 }
	 if (SpringVR_Upperbody.bReceivedOneFrameData) {
		 SpringVR_Upperbody.bReceivedOneFrameData = FALSE;
		 cout << "收到了上身的数据，请处理……" << endl;
	 }
	 if (SpringVR_Lowerbody.bReceivedOneFrameData) {
		 SpringVR_Lowerbody.bReceivedOneFrameData = FALSE;
		 cout << "收到了下身的数据，请处理……" << endl;
	 }
 }
 

 std::cout << "\nShutting down and disconnecting from the MQTT server..." << std::flush;

 //cli.unsubscribe(TOPIC1)->wait();

 cli.stop_consuming();

 cli.disconnect()->wait();

 std::cout << "OK" << std::endl; 
	return 0;

}