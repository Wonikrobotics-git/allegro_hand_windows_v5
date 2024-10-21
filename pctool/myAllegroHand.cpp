// myAllegroHand.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <winsock2.h>
#include <string>
#include <fstream>
#pragma comment (lib, "ws2_32.lib")



#include "stdafx.h"
#include "windows.h"
#include <cstdint>
#include <conio.h>
#include <process.h>
#include <tchar.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include "rPanelManipulatorCmdUtil.h"
#include "BHand/BHand.h"
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT !!
// SET CORRECT HAND PARAMETER HERE BEFORE RUNNING THIS PROGRAM.
const int	HAND_VERSION = 5;
bool	RIGHT_HAND = true;
bool	HAND_TYPE_A = true;
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
const double delT = 0.002;
int CAN_Ch = 0;
bool ioThreadRun = false;
uintptr_t ioThread = 0;
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;
AllegroHand_DeviceMemory_t vars;
int OperatingMode = 0;
double f[4];
double x[4];
double y[4];
double z[4];
int fingertip_sensor[4];
int fingertip_sensor_pre[4];

/////////////////////////////////////////////////////////////////////////////////////////
// for rPanelManipulator
rPanelManipulatorData_t* pSHM = NULL;
double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
BHand* pBHand = NULL;
double q[MAX_DOF];
double q_des[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];



//////////////////////////////////////////////////////////////////
/// for fingertip sensor

int32_t fingertip1 = 0;
int32_t fingertip2 = 0;

/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();

/////////////////////////////////////////////////////////////////////////////////////////
// CAN communication thread
static unsigned int __stdcall ioThreadProc(void* inst)
{
	int id;
	int len;
	unsigned char data[8];
	unsigned char data_return = 0;
	int i;

	while (ioThreadRun)
	{
		/* wait for the event */
		while (0 == get_message(CAN_Ch, &id, &len, data, FALSE))
		{
			switch (id)
			{
			case ID_RTR_HAND_INFO:
			{
				printf(">CAN(%d): AllegroHand hardware version: 0x%02x%02x\n", CAN_Ch, data[1], data[0]);
				printf("                      firmware version: 0x%02x%02x\n", data[3], data[2]);
				printf("                      servo status: %s\n", (data[6] & 0x01 ? "ON" : "OFF"));
			}
			break;
			case ID_RTR_SERIAL:
			{
				printf(">CAN(%d): AllegroHand serial number: %c%c%c%c%c%c%c%c\n", CAN_Ch, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
				printf("                      hand type	   : %c\n", data[2]);
				printf("                      handedness   : %s\n", (data[3] == 'R' ? "Right" : "Left"));

				/* set Hand Type and handedness based on Serial number */
				if (data[2] == 'A') HAND_TYPE_A = true;
				else HAND_TYPE_A = false;
				
				if (data[3] == 'R') RIGHT_HAND = true;
				else RIGHT_HAND = false;
			}
			break;
			case ID_RTR_FINGER_POSE_1:
			case ID_RTR_FINGER_POSE_2:
			case ID_RTR_FINGER_POSE_3:
			case ID_RTR_FINGER_POSE_4:
			{
				int findex = (id & 0x00000007);

				vars.enc_actual[findex * 4 + 0] = (short)(data[0] | (data[1] << 8));
				vars.enc_actual[findex * 4 + 1] = (short)(data[2] | (data[3] << 8));
				vars.enc_actual[findex * 4 + 2] = (short)(data[4] | (data[5] << 8));
				vars.enc_actual[findex * 4 + 3] = (short)(data[6] | (data[7] << 8));
				data_return |= (0x01 << (findex));
				recvNum++;

				if (data_return == (0x01 | 0x02 | 0x04 | 0x08))
				{
					// convert encoder count to joint angle
					for (i = 0; i<MAX_DOF; i++)
						q[i] = ((double)(vars.enc_actual[i])) * (3.141592 / 180.0) * 0.088;
					
					// compute joint torque		
					ComputeTorque();
					
					// set grasping force based on fingertip_sensor sum
					if (OperatingMode == 0) {
					if(fingertip_sensor[0]+ fingertip_sensor[1]+ fingertip_sensor[2]+ fingertip_sensor[3] > 400)
						f[0] = f[1] = f[2] = f[3] = 2;
					else
						f[0] = f[1] = f[2] = f[3] = 1.0;
					}

					// if Geared motor type
					if (!HAND_TYPE_A) {
						tau_des[1] = 0.5 * tau_des[1];
						tau_des[5] = 0.5 * tau_des[5];
						tau_des[9] = 0.5 * tau_des[9];
					}

					// convert desired torque to desired current and PWM count
					for (int i = 0; i < MAX_DOF; i++)
					{
						cur_des[i] = tau_des[i] * 1.43 * 1000; 

						if (cur_des[i] > 240) cur_des[i] = 240;
						else if (cur_des[i] < -240) cur_des[i] = -240;
					}

					// send torques
					for (int i = 0; i < 4; i++)
					{
						vars.pwm_demand[i * 4 + 0] = (short)(cur_des[i * 4 + 0]);
						vars.pwm_demand[i * 4 + 1] = (short)(cur_des[i * 4 + 1]);
						vars.pwm_demand[i * 4 + 2] = (short)(cur_des[i * 4 + 2]);
						vars.pwm_demand[i * 4 + 3] = (short)(cur_des[i * 4 + 3]);
														
						command_set_torque(CAN_Ch, i, &vars.pwm_demand[4 * i]);
					}
							
					sendNum++;
					curTime += delT;
					data_return = 0;
				}
			}
			break;
			case ID_CMD_FINGERTIP_1:
			case ID_CMD_FINGERTIP_3:
			{
				int findex = (id & 0x00000007);
				fingertip1 = (int32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
				fingertip2 = (int32_t)((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);
				
				fingertip_sensor[findex] = fingertip1;
				fingertip_sensor[findex + 1] = fingertip2;

				// low-pass filter for fingertip_sensor
				float alpha = 0.25;
				for (int i = 0; i < 4; ++i) {
					if ((fingertip_sensor[i] < 0) || (fingertip_sensor[i] > 3000))
						fingertip_sensor[i] = 0;

					fingertip_sensor[i] = alpha * fingertip_sensor[i] + (1 - alpha) * fingertip_sensor_pre[i];
					fingertip_sensor_pre[i] = fingertip_sensor[i];

				}

			}
			break;

			default:
				printf(">CAN(%d): unknown command %d, len %d\n", CAN_Ch, id, len);
				/*for(int nd=0; nd<len; nd++)
				printf("%d \n ", data[nd]);*/
				//return;
			}
		}
	}
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events
void MainLoop()
{
	bool bRun = true;

	while (bRun)
	{

		if (!_kbhit())
		{
			Sleep(5);
			if (pSHM)
			{
				switch (pSHM->cmd.command)
				{
				case CMD_SERVO_ON:
					break;
				case CMD_SERVO_OFF:
					if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
					break;
				case CMD_CMD_1:
					if (pBHand) pBHand->SetMotionType(eMotionType_HOME);
					break;
				case CMD_CMD_3:
					if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_3);
					break;
				case CMD_CMD_4:
					if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_4);
					break;
				case CMD_CMD_5:
					if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_IT);
					break;
				case CMD_CMD_6:
					if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_MT);
					break;
				case CMD_CMD_7:
					if (pBHand) pBHand->SetMotionType(eMotionType_ENVELOP);
					break;
				case CMD_CMD_8:
					if (pBHand) pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
					break;
				case CMD_EXIT:
					bRun = false;
					break;
				}
				pSHM->cmd.command = CMD_NULL;
				for (int i=0; i<MAX_DOF; i++)
				{
					pSHM->state.slave_state[i].position = q[i];
					pSHM->cmd.slave_command[i].torque = tau_des[i];
				}
				pSHM->state.time = curTime;
			}
		}
		else
		{
			int c = _getch();
			switch (c)
			{
			case 'q':
				if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
				OperatingMode = 0;
				bRun = false;
				break;
			
			case 'h':
				if (pBHand) pBHand->SetMotionType(eMotionType_HOME);
				OperatingMode = 0;
				command_place(CAN_Ch);
				break;
			
			case 'g':
				if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_3);
				OperatingMode = 0;
				command_pick(CAN_Ch);
				break;

			case 'k':
				if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_4);
				OperatingMode = 0;
				command_pick(CAN_Ch);
				break;
			
			case 'p':
				if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_IT);
				OperatingMode = 0;
				command_pick(CAN_Ch);
				break;
			
			case 'm':
				if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_MT);
				OperatingMode = 0;
				command_pick(CAN_Ch);
				break;
			
			case 'a':
				if (pBHand) pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
				OperatingMode = 0;
				command_place(CAN_Ch);
				break;

			case 'e':
				if (pBHand) pBHand->SetMotionType(eMotionType_ENVELOP);
				OperatingMode = 0;
				break;

			case 'f':
				if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
				OperatingMode = 0;
				command_place(CAN_Ch);
				break;

			}
		}

	}

}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using BHand library
void ComputeTorque()
{
	if (!pBHand) return;
	pBHand->SetJointPosition(q); // tell BHand library the current joint positions
	pBHand->SetJointDesiredPosition(q_des);
	pBHand->UpdateControl(0.002);
	pBHand->SetGraspingForce(f);
	pBHand->GetFKResult(x, y, z);
	pBHand->GetJointTorque(tau_des);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN()
{
	int ret;
	
#if defined(PEAKCAN)
	CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
	CAN_Ch = 1;
#elif defined(SOFTINGCAN)
	CAN_Ch = 1;
#elif defined(NICAN)
	CAN_Ch = 0;
#else
	CAN_Ch = 1;
#endif
	printf("CAN Communication Process\n");
	printf(">CAN(%d): open\n", CAN_Ch);
	ret = command_can_open(CAN_Ch);
	if(ret < 0)
	{
		printf("ERROR command_canopen !!! \n");
		return false;
	}

	recvNum = 0;
	sendNum = 0;
	statTime = 0.0;

	ioThreadRun = true;
	ioThread = _beginthreadex(NULL, 0, ioThreadProc, NULL, 0, NULL);
	printf(">CAN: starts listening CAN frames\n");
	
	// query h/w information
	printf(">CAN: query system information\n");
	ret = request_hand_information(CAN_Ch);
	if (ret < 0)
	{
		printf("ERROR request_hand_information !!! \n");
		command_can_close(CAN_Ch);
		return false;
	}
	ret = request_hand_serial(CAN_Ch);
	if (ret < 0)
	{
		printf("ERROR request_hand_serial !!! \n");
		command_can_close(CAN_Ch);
		return false;
	}

	// set periodic communication parameters(period)
	printf(">CAN: Comm period set\n");
	short comm_period[3] = { 2, 0, 0 }; // millisecond {position, imu, temperature}
	ret = command_set_period(CAN_Ch, comm_period);
	if (ret < 0)
	{
		printf("ERROR command_set_period !!! \n");
		command_can_close(CAN_Ch);
		return false;
	}

	// servo on
	printf(">CAN: servo on\n");
	ret = command_servo_on(CAN_Ch);
	if (ret < 0)
	{
		printf("ERROR command_servo_on !!! \n");
		command_set_period(CAN_Ch, 0);
		command_can_close(CAN_Ch);
		return false;
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN()
{
	int ret;

	printf(">CAN: stop periodic communication\n");
	ret = command_set_period(CAN_Ch, 0);
	if(ret < 0)
	{
		printf("ERROR command_can_stop !!! \n");
	}

	if (ioThreadRun)
	{
		printf(">CAN: stoped listening CAN frames\n");
		ioThreadRun = false;
		WaitForSingleObject((HANDLE)ioThread, INFINITE);
		CloseHandle((HANDLE)ioThread);
		ioThread = 0;
	}

	printf(">CAN(%d): close\n", CAN_Ch);
	ret = command_can_close(CAN_Ch);
	if(ret < 0) printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm()
{
	// Set algorithm based on handedness
	if (RIGHT_HAND)
		pBHand = bhCreateRightHand();
	else
		pBHand = bhCreateLeftHand();

	if (!pBHand) return false;

	// Set algorithm based on type
	if (HAND_TYPE_A) pBHand->SetMotionType(eMotionType_A);
	else pBHand->SetMotionType(eMotionType_B);

	Sleep(50);

	pBHand->SetMotionType(eMotionType_NONE);
	pBHand->SetTimeInterval(delT);
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm()
{
	if (pBHand)
	{
#ifndef _DEBUG
		delete pBHand;
#endif
		pBHand = NULL;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
// Print program information and keyboard instructions
void PrintInstruction()
{
	printf("--------------------------------------------------\n");
	printf("MyAllegroHand: ");
	if (RIGHT_HAND) printf("Right Hand, v%i.x\n\n", HAND_VERSION); else printf("Left Hand, v%i.x\n\n", HAND_VERSION);
	printf("Keyboard Commands:\n");
	printf("H: Home Position \n");
	printf("G: Three-Finger Grasp\n");
	printf("K: Four-Finger Grasp\n");
	printf("P: Two-finger pinch (index-thumb)\n");
	printf("M: Two-finger pinch (middle-thumb)\n");
	printf("E: Envelop Grasp (all fingers)\n");
	printf("A: Gravity Compensation\n");
	printf("F: Servos OFF (any grasp cmd turns them back on)\n");
	printf("Q: Quit this program\n");
	printf("--------------------------------------------------\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname)
{
	if (!cname) return 0;

	if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
		return 0;
	else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
		return 1;
	else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
		return 2;
	else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
		return 3;
	else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
		return 4;
	else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
		return 5;
	else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
		return 6;
	else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
		return 7;
	else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
		return 8;
	else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
		return 9;
	else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
		return 10;
	else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
		return 11;
	else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
		return 12;
	else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
		return 13;
	else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
		return 14;
	else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
		return 15;
	else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
		return 16;
	else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
		return 17;
	else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
		return 18;
	else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
		return 19;
	else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
		return 20;
	else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
		return 21;
	else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
		return 22;
	else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
		return 23;
	else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
		return 24;
	else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
		return 25;
	else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
		return 26;
	else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
		return 271;
	else
		return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int _tmain(int argc, _TCHAR* argv[])
{
	memset(&vars, 0, sizeof(vars));
	memset(q, 0, sizeof(q));
	memset(q_des, 0, sizeof(q_des));
	memset(tau_des, 0, sizeof(tau_des));
	memset(cur_des, 0, sizeof(cur_des));
	curTime = 0.0;

	pSHM = getrPanelManipulatorCmdMemory();
	
	if (OpenCAN())
	{
		Sleep(50);
		if (CreateBHandAlgorithm())
		{
			PrintInstruction();
			MainLoop();
		}
			
	}

	CloseCAN();
	DestroyBHandAlgorithm();
	closerPanelManipulatorCmdMemory();

	return 0;
}
