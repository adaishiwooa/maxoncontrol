//============================================================================
// Name        : HelloEposCmd.cpp
// Author      : Dawid Sienkiewicz
// Version     :
// Copyright   : maxon motor ag 2014
// Description : Hello Epos in C++, Ansi-style
//============================================================================
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <std_msgs/Int32.h>
#include <time.h>

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
//int* pPositionIs=0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
clock_t start,stop;
double thetime,thetimethisloop;
int g_baudrate = 0;
long realposition=0;
int revcount=0;
float tanrecord=0;

const string g_programName = "HelloEposCmd";

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long targetPosition);
int   Demo(unsigned int* p_pErrorCode);

void PrintUsage()
{
	cout << "Usage: HelloEposCmd -h -n 1 -d deviceName -s protocolStackName -i interfaceName -p portName -b baudrate" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
	cout << message << endl;
}

void SeparatorLine()
{
	const int lineLength = 60;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintSettings()
{
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters()
{
	//USB
	g_usNodeId = 1;
	g_deviceName = "EPOS2"; //EPOS
	g_protocolStackName = "MAXON SERIAL V2"; //MAXON_RS232
	g_interfaceName = "USB"; //RS232
	g_portName = "USB0"; // /dev/ttyS1
	g_baudrate = 1000000; //115200
}

int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int ParseArguments(int argc, char** argv)
{
	int lOption;
	int lResult = MMC_SUCCESS;

	// Shut GetOpt error messages down (return '?'):
	opterr = 0;
	// Retrieve the options:
	while ( (lOption = getopt(argc, argv, ":hd:s:i:p:b:n:")) != -1 )
	{
		switch ( lOption ) {
			case 'h':
				PrintUsage();
				lResult = 1;
				break;
			case 'd':
				g_deviceName = optarg;
				break;
			case 's':
				g_protocolStackName = optarg;
				break;
			case 'i':
				g_interfaceName = optarg;
				break;
			case 'p':
				g_portName = optarg;
				break;
			case 'b':
				g_baudrate = atoi(optarg);
				break;
			case 'n':
				g_usNodeId = (unsigned short)atoi(optarg);
				break;
			case '?':  // unknown option...
				stringstream msg;
				msg << "Unknown option: '" << char(optopt) << "'!";
				LogInfo(msg.str());
				PrintUsage();
				lResult = MMC_FAILED;
				break;
		}
	}

	return lResult;
}

bool DemoProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	//~ msg << "set profile velocity mode, node = " << p_usNodeId;
	printf("\n");

	LogInfo(msg.str());

	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
			long targetvelocity = 800;

			if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				//LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
			}

			sleep(30);
	}

		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt velocity movement");

			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
			}
		}

	return lResult;
}

/*
int Demo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

	//lResult = DemoProfileVelocityMode(g_pKeyHandle, g_usNodeId, lErrorCode);
	lResult = DemoProfilePositionMode(g_pKeyHandle, g_usNodeId, lErrorCode);


			if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
			{
				LogError("VCS_SetDisableState", lResult, lErrorCode);
				lResult = MMC_FAILED;
			}
		
	return lResult;
}
*/

int DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long targetPosition)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		    
			if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 0, 1, &p_rlErrorCode) == 0)
			{
				LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}

			usleep(50000);
			printf("\n");
		}

		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt position movement");

			if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
		}

	return lResult;
}

int GetPositionIs(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int & Real_position, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	
	if(VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &Real_position, &p_rlErrorCode) == 0)
	{
		LogError("Error getting absolute position", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	
	return lResult;
}


int PrepareDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

void PrintHeader()
{
	SeparatorLine();

	LogInfo("Epos Command Library Example Program, (c) maxonmotor ag 2014");

	SeparatorLine();
}

int targetpositioncal(float tan, float yaw)
{
	
	// first calculate the actural position in one revolution from 0 - 180 then -180 - 0
	// in one revolution, if it -180 - 0 range, it should add 360 degree
	if ( tan < 0)
	{
		tan = tan + 360;
	}
	
	
	if (abs(tan-tanrecord) > 300)
	{
		revcount = revcount +1;
		printf("Revolution plus 1");
	}
	
	tanrecord = tan;
	
	float nozzleangle = - tan - yaw ;
	printf("expected angle = %f\n",nozzleangle);
	
	//~ int nozzleposition = int ( - nozzleangle * 78.7 - 28342* revcount);
	//~ int nozzleposition = int ( nozzleangle * 77.55 - 27918* revcount);
	int nozzleposition = int ( nozzleangle * 77.43 - 27875* revcount);
	
	return nozzleposition;
}

void eposCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    unsigned int lErrorCode = 0;
    int absouPosition = 0;
    
	int i = 0;
	int lResult =1;
	//~ long targetPositioninputangle = 0 - (msg->data);
	//~ float inputangle[2] = msg->data;
	float Arr[2];
	
	for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
	
	float tangent = Arr[1];
	printf("tanget = %f\n",tangent);
	float yaw = Arr[0];
	printf("yaw = %f\n",yaw);
	
	
	//~ long targetPositioninput = targetPositioninputangle*79;
	long targetPositioninput = targetpositioncal(tangent, yaw);
	
	lResult = GetPositionIs(g_pKeyHandle, g_usNodeId, absouPosition, lErrorCode);
	
	long targetPosition = targetPositioninput - absouPosition ;
	//~ long targetPosition = targetPositioninput - realposition ;
	
	//~ lResult = GetPositionIs(g_pKeyHandle, g_usNodeId, absouPosition, lErrorCode);
	
	lResult = DemoProfilePositionMode(g_pKeyHandle, g_usNodeId, lErrorCode, targetPosition);
	
	float actualangle = (absouPosition + 27875* revcount)/(-77.43);
	
	printf("the absoulute angle = %f", actualangle);
	
	printf("the position target is %ld\n",targetPositioninput);
	printf("the absoulute position is %d\n", absouPosition);
	
	//~ sleep(1);
	
	realposition = targetPositioninput;
	
	return;
}

int main(int argc, char** argv)
{
	
	
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	unsigned int lErrorCode = 0;

//	PrintHeader();

	SetDefaultParameters();

	
	if((lResult = ParseArguments(argc, argv))!=MMC_SUCCESS)
	{
		return lResult;
	}

//	PrintSettings();

	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo", lResult, ulErrorCode);
		return lResult;
	}
/*
	if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("Demo", lResult, ulErrorCode);
		return lResult;
	}
*/


	ros::init(argc, argv, "epostest");
	ros::NodeHandle n;	
	
	ros::Subscriber sub = n.subscribe("/Extruder_angles", 1, eposCallback);

	ros::spin();
	
	if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
	{
		LogError("VCS_SetDisableState", lResult, lErrorCode);
		lResult = MMC_FAILED;
	}
	
	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}


	return 1;
}
