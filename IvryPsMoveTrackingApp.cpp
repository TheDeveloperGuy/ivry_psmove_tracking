/*************************************************************************
*
* Copyright (C) 2016-2024 Mediator Software and/or its subsidiary(-ies).
* All rights reserved.
* Contact: Mediator Software (info@mediator-software.com)
*
* NOTICE:  All information contained herein is, and remains the property of
* Mediator Software and its suppliers, if any.
* The intellectual and technical concepts contained herein are proprietary
* to Mediator Software and its suppliers and may be covered by U.S. and
* Foreign Patents, patents in process, and are protected by trade secret or
* copyright law. Dissemination of this information or reproduction of this
* material is strictly forbidden unless prior written permission is obtained
* from Mediator Software.
*
* If you have questions regarding the use of this file, please contact
* Mediator Software (info@mediator-software.com).
*
***************************************************************************/

#include "IvryPsMoveTrackingApp.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define PSMOVE_SERVICE_PROCESSES "PSMoveService.exe\0PSMoveServiceAdmin.exe\0"
#define PSMOVE_SERVICE_EXE_PATH "\\PSMoveServiceEx\\PSMoveService.exe"

#undef UNICODE
#include <Tlhelp32.h>
#include <process.h>

/** Convert Quaternion to Euler **/
static inline void HmdQuaternion_EulerFromQuaternion(double(&euler)[3], const vr::HmdQuaternion_t &quaternion)
{
	double w = quaternion.w;
	double x = quaternion.x;
	double y = quaternion.y;
	double z = quaternion.z;

	double test = x*y + z*w;
	if (test >= 0.5f)
	{
		euler[1] = 2 * atan2(x, w);
		euler[2] = M_PI_2;
		euler[0] = 0;
		return;
	}
	if (test <= -0.5f)
	{
		euler[1] = -2 * atan2(x, w);
		euler[2] = -M_PI_2;
		euler[0] = 0;
		return;
	}

	double sqx = x*x;
	double sqy = y*y;
	double sqz = z*z;
	euler[1] = atan2(2 * y*w - 2 * x*z, 1 - 2 * sqy - 2 * sqz);
	euler[2] = asin(2 * test);
	euler[0] = atan2(2 * x*w - 2 * y*z, 1 - 2 * sqx - 2 * sqz);
}

/** Convert Euler to Quaternion **/
static inline void HmdQuaternion_QuaternionFromEuler(vr::HmdQuaternion_t &quaternion, const double(&euler)[3])
{
	double c1 = cos(euler[1] / 2);
	double s1 = sin(euler[1] / 2);
	double c2 = cos(euler[2] / 2);
	double s2 = sin(euler[2] / 2);
	double c3 = cos(euler[0] / 2);
	double s3 = sin(euler[0] / 2);
	double c1c2 = c1*c2;
	double s1s2 = s1*s2;

	quaternion.w = c1c2*c3 - s1s2*s3;
	quaternion.x = c1c2*s3 + s1s2*c3;
	quaternion.y = s1*c2*c3 + c1*s2*s3;
	quaternion.z = c1*s2*c3 - s1*c2*s3;
}

IvryPsMoveTrackingApp::IvryPsMoveTrackingApp()
	: m_pPsMoveClient(NULL)
	, m_hPsMoveService(NULL)
	, m_bTrackingEnabled(false)
	, m_bUseDeviceOrientation(true)
	, m_fDeviceCenterOffset(0.0f)
	, m_bDeviceWasRecentered(true)
{
}

IvryPsMoveTrackingApp::~IvryPsMoveTrackingApp()
{
	if (m_pPsMoveClient != NULL)
	{
		m_pPsMoveClient->close();
		delete m_pPsMoveClient;
	}
}

/** Run tracker **/
DWORD IvryPsMoveTrackingApp::Run()
{
	DWORD result = ERROR_SUCCESS;

	// Open connection to driver
	if (Open())
	{
		// PSMoveService not running?
		if (!IsPsMoveServiceRunning())
		{
			// Get path to this executable
			char szPath[MAX_PATH] = { 0 };
			::GetModuleFileNameA(NULL, szPath, MAX_PATH);

			// Remove EXE name
			char *pos = &szPath[strlen(szPath)];
			while (pos > szPath && *--pos != '\\');
			*pos = '\0';

			// Remove arch folder
			while (pos > szPath && *--pos != '\\');
			*pos = '\0';
			// Remove bin folder
			while (pos > szPath && *--pos != '\\');
			*pos = '\0';

			// Add path to PSMoveService exectable
			strcat(szPath, PSMOVE_SERVICE_EXE_PATH);
			if (::GetFileAttributesA(szPath) != INVALID_FILE_ATTRIBUTES)
			{
				STARTUPINFOA sInfoProcess = { 0 };
				sInfoProcess.cb = sizeof(STARTUPINFOA);
				sInfoProcess.dwFlags = STARTF_USESHOWWINDOW;
				sInfoProcess.wShowWindow = SW_HIDE;
				PROCESS_INFORMATION pInfoStartedProcess;
				if (::CreateProcessA(NULL, (LPSTR)szPath, NULL, NULL, FALSE, 0, NULL, NULL, &sInfoProcess, &pInfoStartedProcess))
				{
					m_hPsMoveService = pInfoStartedProcess.hProcess;

					LogMessage("Started PSMoveService process\n");

					// Wait for service to start up
					::Sleep(1000);
				}
			}
		}

		m_pPsMoveClient = new IvryPsMoveClient(this);
		if (m_pPsMoveClient != NULL)
		{
			// Start PSMoveClient tracking
			if (m_pPsMoveClient->open())
			{
				// Disable PSVR Leds
				EnableDeviceLeds(false);

				// Disable drift correction
				EnableDeviceDriftCorrection(false);

				// Run tracking
				m_pPsMoveClient->run();

				// Disable tracking
				TrackingEnabled(false);

				// Enable PSVR Leds
				EnableDeviceLeds(true);

				// Enable device orientation
				EnableDeviceOrientation(true);
			}
			else
			{
				result = m_pPsMoveClient->get_last_error();
			}

			delete m_pPsMoveClient;
			m_pPsMoveClient = NULL;
		}
		else
		{
			result = ERROR_OUTOFMEMORY;
		}

		if (result != ERROR_SUCCESS)
		{
			char error[256];
			sprintf(error, "Could not start PSMoveService tracking (%d)!\n", result);
			LogMessage(error);
		}

		// PSMoveService process started here?
		if (m_hPsMoveService != NULL)
		{
			// Terminate it
			::TerminateProcess(m_hPsMoveService, 0);
			m_hPsMoveService = NULL;
		}

		// Close connection to driver
		Close();
	}
	else
	{
		result = GetLastError();
	}

	return result;
}

/** Check if PSMoveService is running **/
bool IvryPsMoveTrackingApp::IsPsMoveServiceRunning()
{
	int retries = 3;
	bool running = false;

	// Check if process is PSMoveService
	const char *processes = PSMOVE_SERVICE_PROCESSES;
	while (*processes != 0)
	{
		// Get snapshot of running processes
		HANDLE hSnapShot = ::CreateToolhelp32Snapshot(TH32CS_SNAPALL, NULL);
		PROCESSENTRY32 pEntry;
		pEntry.dwSize = sizeof(pEntry);
		BOOL hRes = ::Process32First(hSnapShot, &pEntry);
		while (hRes)
		{
			// PSMoveService process?
			if (_stricmp(pEntry.szExeFile, processes) == 0)
			{
				running = true;
				break;
			}

			hRes = ::Process32Next(hSnapShot, &pEntry);
		}

		::CloseHandle(hSnapShot);

		processes += strlen(processes) + 1;
	}

	return running;
}

/** Pose has been recevied from driver **/
void IvryPsMoveTrackingApp::OnDevicePoseUpdated(const vr::DriverPose_t &pose)
{
	if (m_bDeviceWasRecentered)
	{
		double euler[3];
		HmdQuaternion_EulerFromQuaternion(euler, pose.qRotation);

		m_fDeviceCenterOffset = euler[1];
		m_bDeviceWasRecentered = false;
	}
}

/** Get min/max tracking rates (in Hz) **/
float IvryPsMoveTrackingApp::GetMinTrackingRate()
{
	return PSMOVE_DEFAULT_TRACKING_RATE;
}

float IvryPsMoveTrackingApp::GetMaxTrackingRate()
{
	return PSMOVE_DEFAULT_TRACKING_RATE;
}

/** Get/Set tracking rate (in Hz) **/
float IvryPsMoveTrackingApp::GetTrackingRate()
{
	return PSMOVE_DEFAULT_TRACKING_RATE;
}

void IvryPsMoveTrackingApp::SetTrackingRate(float rate)
{

}

/** Device orientation has been enabled/disabled by user **/
void IvryPsMoveTrackingApp::OnDeviceOrientationEnabled(bool enable)
{
	m_bUseDeviceOrientation = enable;
}

/** Driver has recentered headset **/
void IvryPsMoveTrackingApp::OnDeviceRecenter()
{
	m_bDeviceWasRecentered = true;
}

/** Reload tracker settings **/
void IvryPsMoveTrackingApp::OnReloadSettings()
{
	if (m_pPsMoveClient != NULL)
	{
		m_pPsMoveClient->load_settings();
	}
}

/** Driver is requesting tracking process quit **/
void IvryPsMoveTrackingApp::OnQuit()
{
	if (m_pPsMoveClient != NULL)
	{
		// Shut down tracking
		m_pPsMoveClient->enable_tracking(false);

		LogMessage("Shutting down\n");
	}
}

void IvryPsMoveTrackingApp::PoseUpdated(const vr::DriverPose_t &pose)
{
	// Tracking not enabled yet?
	if (!m_bTrackingEnabled)
	{
		// Enable tracking
		TrackingEnabled(true);
		m_bTrackingEnabled = true;
	}

	// Send pose to driver
	IvryTrackingApp::PoseUpdated(pose);
}

/** Controller haptics request has been received from driver **/
void IvryPsMoveTrackingApp::OnControllerHaptics(uint32_t id, uint32_t component, float fDurationSeconds, float fFrequency, float fAmplitude)
{
	if (m_pPsMoveClient != NULL)
	{
		m_pPsMoveClient->controller_haptics(id, fDurationSeconds, fFrequency, fAmplitude);
	}
}
