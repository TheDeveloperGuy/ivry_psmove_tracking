/*************************************************************************
*
* Copyright (C) 2016-2020 Mediator Software and/or its subsidiary(-ies).
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

/*
* Some portions edited from FreepieMoveClient.cpp in PSMoveFreepieBridge
*/

#include <tchar.h>
#include "IvryPsMoveClient.h"
#include <Tlhelp32.h>
#include <shlwapi.h>
#include "shlobj.h"

/** Imports **/
#pragma comment(lib,"shlwapi.lib")

#define PSMOVE_SERVICE_EXECUTABLE_NAME TEXT("PSMoveService.exe")

#define PSMOVE_BRIDGE_SETTINGS_NAME TEXT("\\PSMoveSteamVRBridge\\PSMoveSteamVRBridgeConfig.json")
#define PSMOVE_BRIDGE_FILTER_SERIAL "filter_virtual_hmd_serial"

#pragma comment(lib, "PSMoveClient_CAPI.lib")

#undef min
#undef max
#include "document.h"
#include "stringbuffer.h"
#include "prettywriter.h"
using namespace rapidjson;

static inline vr::HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	vr::HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

IvryPsMoveClient::IvryPsMoveClient(
	eDeviceType deviceType,
	int32_t deviceCount,
	int32_t deviceIDs[],
	PSMTrackingColorType bulbColors[],
	bool sendSensorData,
	int triggerAxisIndex)
	: _sensorThread(INVALID_HANDLE_VALUE)
	, _sensorTimer(INVALID_HANDLE_VALUE)
	, _sensorActive(FALSE)
	, _isOpen(false)
	, _trackingRate(PSMOVE_DEFAULT_TRACKING_RATE)
	, _trackingCallback(NULL)
	, _trackingCallbackContext(NULL)
	, _controllerCallback(NULL)
	, _controllerCallbackContext(NULL)
	, _logCallback(NULL)
	, _logCallbackContext(NULL)
{
	m_device_type = deviceType;

	memset(trackedHmdIDs, 0, sizeof(trackedHmdIDs));
	memset(trackedControllerIDs, 0, sizeof(trackedControllerIDs));
	if (deviceType == _deviceTypeHMD)
	{
		for (int i = 0; i < deviceCount && i < PSMOVE_MAX_CONTROLLERS; i++)
		{
			trackedHmdIDs[i] = deviceIDs[i];
		}
		trackedHmdCount = deviceCount <= PSMOVE_MAX_CONTROLLERS ? deviceCount : PSMOVE_MAX_CONTROLLERS;
	}
	else
	{
		for (int i = 0; i < deviceCount && i < PSMOVE_MAX_CONTROLLERS; i++)
		{
			trackedControllerIDs[i] = deviceIDs[i];
		}
		trackedControllerCount = deviceCount <= PSMOVE_MAX_CONTROLLERS ? deviceCount : PSMOVE_MAX_CONTROLLERS;
	}

	memset(trackedBulbColors, 0, sizeof(trackedBulbColors));
	for (int i = 0; i < deviceCount && i < PSMOVE_MAX_CONTROLLERS; i++)
	{
		trackedBulbColors[i] = bulbColors[i];
	}

	m_sendSensorData = sendSensorData;
	m_triggerAxisIndex = triggerAxisIndex;

	for (int i = 0; i < PSMOVE_MAX_CONTROLLERS; i++)
	{
		hmd_views[i] = nullptr;
		controller_views[i] = nullptr;
		start_stream_request_ids[i] = -1;
	}
}

IvryPsMoveClient::~IvryPsMoveClient()
{
	if (_isOpen)
	{
		close();
	}
}

bool IvryPsMoveClient::open()
{
	// Get path to user's roaming app data
	TCHAR configPath[MAX_PATH];
	if (SUCCEEDED(SHGetFolderPath(NULL, CSIDL_APPDATA, NULL, 0, configPath)))
	{
		// Add PSMoveSteamVRBridge settings file path
		_tcscat(configPath, PSMOVE_BRIDGE_SETTINGS_NAME);

		// Open file
		HANDLE file = CreateFile(configPath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
		if (file != INVALID_HANDLE_VALUE)
		{
			// Get size
			DWORD size = GetFileSize(file, NULL);
			if (size != INVALID_FILE_SIZE)
			{
				// Allocate buffer for contents
				char* contents = new char[size + 1];
				if (size != NULL)
				{
					// Read contents
					DWORD bytes;
					if (ReadFile(file, contents, size, &bytes, NULL)
						&& bytes == size)
					{
						// Null terminate
						contents[size] = 0;

						// Contents are valid JSON?
						Document data;
						if (!data.Parse(contents).HasParseError()
							&& data.HasMember(PSMOVE_BRIDGE_FILTER_SERIAL))
						{
							// Get value of virtual HMD filter address
							const Value& value = data[PSMOVE_BRIDGE_FILTER_SERIAL];
							if (value.IsString() && value.GetStringLength() > 0)
							{
								// Copy serial number, converting '_' to ':'
								_controllerSerialNoFilter.clear();
								for (const char* serialNo = value.GetString(); *serialNo; serialNo++)
								{
									if (*serialNo != '_') _controllerSerialNoFilter += *serialNo;
									else _controllerSerialNoFilter += ':';
								}
							}
						}
					}

					delete[] contents;
				}
			}

			CloseHandle(file);
		}
	}

	try
	{
		// Attempt to start the client
		if (startup())
		{
			_isOpen = true;
			return true;
		}
		else
		{
			log_msg("Failed to startup PSMove Client\n");
		}
	}
	catch (std::exception& e)
	{
		log_msg("Exception: %s\n", e.what());
	}

	try
	{
		// Attempt to shutdown the client
		shutdown();
	}
	catch (std::exception& e)
	{
		log_msg("Exception: %s\n", e.what());
	}

	return false;
}

void IvryPsMoveClient::close()
{
	enable_tracking(false);
	shutdown();
	_isOpen = false;
}

bool IvryPsMoveClient::enable_tracking(bool enable)
{
	if (enable)
	{
		if (_sensorThread == INVALID_HANDLE_VALUE)
		{
			_sensorThread = ::CreateThread(NULL, 0, sensor_thread_proc, (void*)this, 0, NULL);

			return (_sensorThread != INVALID_HANDLE_VALUE);
		}

		return true;
	}
	else
	{
		if (_sensorThread != INVALID_HANDLE_VALUE)
		{
			_sensorActive = FALSE;
			::WaitForSingleObject(_sensorThread, INFINITE);

			return true;
		}
	}

	return false;
}

bool IvryPsMoveClient::is_psmoveservice_running()
{
	bool isRunning = false;

	HANDLE hSnapShot = CreateToolhelp32Snapshot(TH32CS_SNAPALL, NULL);
	PROCESSENTRY32 pEntry;
	pEntry.dwSize = sizeof(pEntry);
	BOOL hRes = Process32First(hSnapShot, &pEntry);
	while (hRes)
	{
		if (_tcsicmp(pEntry.szExeFile, PSMOVE_SERVICE_EXECUTABLE_NAME) == 0)
		{
			isRunning = true;
			break;
		}
		hRes = Process32Next(hSnapShot, &pEntry);
	}
	CloseHandle(hSnapShot);

	return isRunning;
}

void IvryPsMoveClient::run()
{
	_sensorActive = TRUE;

	_sensorTimer = ::CreateWaitableTimer(NULL, TRUE, NULL);
	if (_sensorTimer != NULL)
	{
		// Not exiting or deactivating?
		while (_sensorActive)
		{
			// Wait until it's time for next sensor poll
			DWORD interval = (DWORD)(1000000.0f / _trackingRate);
			LARGE_INTEGER dueTime;
			dueTime.QuadPart = interval * -10;
			::SetWaitableTimer(_sensorTimer, &dueTime, 0, NULL, NULL, FALSE);
			::WaitForSingleObject(_sensorTimer, (DWORD)(interval / 1000));

			// Update tracking
			update();
		}

		::CloseHandle(_sensorTimer);
		_sensorTimer = INVALID_HANDLE_VALUE;
	}
	else
	{
		log_msg("ERROR: Could not create PSVR sensor polling timer (%08x)!\n", ::GetLastError());
	}
}

DWORD WINAPI IvryPsMoveClient::sensor_thread_proc(void *arg)
{
	IvryPsMoveClient *context = (IvryPsMoveClient*)arg;
	if (context != NULL)
	{
		context->run();
		context->_sensorThread = INVALID_HANDLE_VALUE;
	}

	return 0;
}

void IvryPsMoveClient::handle_client_psmove_event(PSMEventMessage::eEventType event_type)
{
	switch (event_type)
	{
	case PSMEventMessage::PSMEvent_connectedToService:
		log_msg("Connected to service\n");
		PSMHmdList hmdList;
		memset(&hmdList, 0, sizeof(hmdList));

		PSM_GetHmdList(&hmdList, PSM_DEFAULT_TIMEOUT);
		if (hmdList.count > 0)
		{
			log_msg("Found %d virtual headset(s)\n", hmdList.count);

			if (hmdList.count >= 1)
			{
				trackedHmdIDs[0] = hmdList.hmd_id[0];
				trackedHmdCount = 1;
				m_device_type = _deviceTypeHMD;
			}
		}
		if (hmdList.count == 0)
		{
			PSMControllerList controllerList;
			memset(&controllerList, 0, sizeof(controllerList));
			if (PSM_GetControllerList(&controllerList, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
			{
				log_msg("Found %d controller(s)\n", controllerList.count);

				bool found = false;
				if (_controllerSerialNoFilter.length() > 0)
				{
					for (int i = 0; i < controllerList.count; i++)
					{
						if (_controllerSerialNoFilter.compare(controllerList.controller_serial[i]) == 0)
						{
							log_msg("Using specified controller id:%d (%s) as HMD tracker\n", 
								controllerList.controller_id[i], controllerList.controller_serial[i]);
							trackedControllerIDs[0] = controllerList.controller_id[i];
							trackedControllerCount = 1;
							m_device_type = _deviceTypeController;
							found = true;
							break;
						}
					}
				}

				if (!found && controllerList.count > 2)
				{
					log_msg("Using controller id:%d (%s) as HMD tracker\n", 
						controllerList.controller_id[2], controllerList.controller_serial[2]);
					_controllerSerialNoFilter = controllerList.controller_serial[2];
					trackedControllerIDs[0] = controllerList.controller_id[2];
					trackedControllerCount = 1;
					m_device_type = _deviceTypeController;
				}
			}
		}

		if (m_device_type == _deviceTypeHMD)
		{
			init_hmd_views();
		}
		else
		{
			init_controller_views();
		}
		break;
	case PSMEventMessage::PSMEvent_failedToConnectToService:
		log_msg("Failed to connect to service\n");
		break;
	case PSMEventMessage::PSMEvent_disconnectedFromService:
		log_msg("Disconnected from service\n");
		break;
	case PSMEventMessage::PSMEvent_opaqueServiceEvent:
		log_msg("Opaque service event(%d). Ignored.\n", static_cast<int>(event_type));
		break;
	case PSMEventMessage::PSMEvent_controllerListUpdated:
		if (m_device_type == _deviceTypeController)
		{
			log_msg("Controller list updated. Reinitializing controller views.\n");
			free_controller_views();
			init_controller_views();
		}
		else
		{
			log_msg("Controller list updated. Ignored.\n");
		}
		break;
	case PSMEventMessage::PSMEvent_trackerListUpdated:
		log_msg("Tracker list updated. Ignored.\n");
		break;
	case PSMEventMessage::PSMEvent_hmdListUpdated:        
		if (m_device_type == _deviceTypeHMD)
		{
			log_msg("HMD list updated. Reinitializing HMD views.\n");
			free_hmd_views();
			init_hmd_views();
		}
		else
		{
			log_msg("HMD list updated. Ignored.\n");
		}
		break;
	case PSMEventMessage::PSMEvent_systemButtonPressed:
		log_msg("System button pressed. Ignored.\n");
		break;
	default:
		log_msg("unhandled event(%d)\n", static_cast<int>(event_type));
		break;
	}
}

void IvryPsMoveClient::handle_acquire_hmd(PSMResult resultCode, PSMHmdID trackedHmdIndex)
{
	if (resultCode == PSMResult_Success)
	{
		log_msg("Acquired HMD %d\n", hmd_views[trackedHmdIndex]->HmdID);
	}
	else
	{
		log_msg("failed to acquire HMD \n");
	}
}

void IvryPsMoveClient::handle_acquire_controller(PSMResult resultCode, PSMControllerID trackedControllerIndex)
{
	if (resultCode == PSMResult_Success)
	{
		log_msg("Acquired controller %d\n", controller_views[trackedControllerIndex]->ControllerID);
	}
	else
	{
		log_msg("failed to acquire controller \n");
	}
}

bool IvryPsMoveClient::startup()
{
	bool success = is_psmoveservice_running();

	// Attempt to connect to the server
	if (success)
	{
		if (PSM_InitializeAsync("localhost", "9512") != PSMResult_RequestSent)
		{
			log_msg("Failed to initialize the client network manager\n");
			_lastError = ERROR_APP_INIT_FAILURE;
			success = false;
		}
	}
	else
	{
		_lastError = ERROR_NOT_READY;
	}

	if (success)
	{
		last_report_fps_timestamp =
			std::chrono::duration_cast< std::chrono::milliseconds >(
				std::chrono::system_clock::now().time_since_epoch());
		_lastError = ERROR_SUCCESS;
	}

	return success;
}

void IvryPsMoveClient::update()
{
	// Process incoming/outgoing networking requests
	PSM_UpdateNoPollMessages();

	// Poll events queued up by the call to PSM_UpdateNoPollMessages()
	PSMMessage message;
	while (PSM_PollNextMessage(&message, sizeof(message)) == PSMResult_Success)
	{
		switch (message.payload_type)
		{
		case PSMMessage::_messagePayloadType_Response:
            {
                int trackedDeviceCount= (m_device_type == _deviceTypeHMD) ? trackedHmdCount : trackedControllerCount;

			    for (int i = 0; i < trackedDeviceCount; i++)
			    {
				    if (start_stream_request_ids[i] != -1 &&
					    message.response_data.request_id == start_stream_request_ids[i])
				    {
					    if (m_device_type == _deviceTypeHMD)
					    {
						    handle_acquire_hmd(message.response_data.result_code, i);
					    }
					    else
					    {
						    handle_acquire_controller(message.response_data.result_code, i);
					    }
					    start_stream_request_ids[i] = -1;
				    }
			    }
            }
			break;
		case PSMMessage::_messagePayloadType_Event:
			handle_client_psmove_event(message.event_data.event_type);
			break;
		}
	}

	// Update tracking?
	if (_trackingCallback != NULL)
	{
		if (m_device_type == _deviceTypeHMD)
		{
			for (int i = 0; i < trackedHmdCount; i++)
			{
				if (hmd_views[i] && hmd_views[i]->bValid)
				{
					std::chrono::milliseconds now =
						std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::system_clock::now().time_since_epoch());
					std::chrono::milliseconds diff = now - last_report_fps_timestamp;

					PSMPosef hmdPose;
					if (PSM_GetHmdPose(hmd_views[i]->HmdID, &hmdPose) == PSMResult_Success)
					{
						vr::DriverPose_t pose;
						memset(&pose, 0, sizeof(pose));
						pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
						pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
						pose.qRotation = HmdQuaternion_Init(1, 0, 0, 0);
						pose.poseIsValid = true;
						pose.result = vr::TrackingResult_Running_OK;
						pose.deviceIsConnected = true;

						pose.vecPosition[0] = hmdPose.Position.x * PSMOVE_SCALE_FACTOR;
						pose.vecPosition[1] = hmdPose.Position.y * PSMOVE_SCALE_FACTOR;
						pose.vecPosition[2] = hmdPose.Position.z * PSMOVE_SCALE_FACTOR;

						if (hmd_views[i]->HmdType == PSMHmd_Morpheus)
						{
							PSMQuatf normalizedQuat = PSM_QuatfNormalizeWithDefault(&hmdPose.Orientation, k_psm_quaternion_identity);
							pose.qRotation.w = normalizedQuat.w;
							pose.qRotation.x = normalizedQuat.x;
							pose.qRotation.y = normalizedQuat.y;
							pose.qRotation.z = normalizedQuat.z;

							if (m_sendSensorData)
							{
								PSMMorpheusCalibratedSensorData sensors = hmd_views[i]->HmdState.MorpheusState.CalibratedSensorData;

								pose.vecAcceleration[0] = sensors.Accelerometer.x;
								pose.vecAcceleration[1] = sensors.Accelerometer.y;
								pose.vecAcceleration[2] = sensors.Accelerometer.z;

								pose.vecAngularAcceleration[0] = sensors.Gyroscope.x;
								pose.vecAngularAcceleration[1] = sensors.Gyroscope.y;
								pose.vecAngularAcceleration[2] = sensors.Gyroscope.z;
							}
						}

						// First HMD only
						if (i == 0)
						{
							// Update tracking
							_trackingCallback(pose, _trackingCallbackContext);
						}
					}
				}
			}
		}
		else
		{
			for (int i = 0; i < trackedControllerCount; i++)
			{
				if (controller_views[i] && controller_views[i]->bValid &&
					(controller_views[i]->ControllerType == PSMController_Move ||
					controller_views[i]->ControllerType == PSMController_Virtual))
				{
					std::chrono::milliseconds now =
						std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::system_clock::now().time_since_epoch());
					std::chrono::milliseconds diff = now - last_report_fps_timestamp;

					vr::DriverPose_t pose;
					memset(&pose, 0, sizeof(pose));
					pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
					pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
					pose.poseIsValid = true;
					pose.result = vr::TrackingResult_Running_OK;
					pose.deviceIsConnected = true;

					PSMPosef controllerPose;
					PSM_GetControllerPose(controller_views[i]->ControllerID, &controllerPose);

					pose.vecPosition[0] = controllerPose.Position.x * PSMOVE_SCALE_FACTOR;
					pose.vecPosition[1] = controllerPose.Position.y * PSMOVE_SCALE_FACTOR;
					pose.vecPosition[2] = controllerPose.Position.z * PSMOVE_SCALE_FACTOR;

					PSMQuatf normalizedQuat = PSM_QuatfNormalizeWithDefault(&controllerPose.Orientation, k_psm_quaternion_identity);
					pose.qRotation.w = normalizedQuat.w;
					pose.qRotation.x = normalizedQuat.x;
					pose.qRotation.y = normalizedQuat.y;
					pose.qRotation.z = normalizedQuat.z;

					uint16_t buttonsPressed = 0;
					float triggerValue = 0.0f;

					if (controller_views[i]->ControllerType == PSMController_Move)
					{
						if (m_sendSensorData)
						{
							const PSMPSMove &moveView = controller_views[i]->ControllerState.PSMoveState;
							const PSMPSMoveCalibratedSensorData &sensors = moveView.CalibratedSensorData;

							pose.vecAcceleration[0] = sensors.Accelerometer.x;
							pose.vecAcceleration[1] = sensors.Accelerometer.y;
							pose.vecAcceleration[2] = sensors.Accelerometer.z;

							pose.vecAngularAcceleration[0] = sensors.Gyroscope.x;
							pose.vecAngularAcceleration[1] = sensors.Gyroscope.y;
							pose.vecAngularAcceleration[2] = sensors.Gyroscope.z;
						}

						const PSMPSMove &moveView = controller_views[i]->ControllerState.PSMoveState;

						buttonsPressed |= (moveView.SquareButton == PSMButtonState_DOWN || moveView.SquareButton == PSMButtonState_PRESSED);
						buttonsPressed |= ((moveView.TriangleButton == PSMButtonState_DOWN || moveView.TriangleButton == PSMButtonState_PRESSED) << 1);
						buttonsPressed |= ((moveView.CrossButton == PSMButtonState_DOWN || moveView.CrossButton == PSMButtonState_PRESSED) << 2);
						buttonsPressed |= ((moveView.CircleButton == PSMButtonState_DOWN || moveView.CircleButton == PSMButtonState_PRESSED) << 3);
						buttonsPressed |= ((moveView.MoveButton == PSMButtonState_DOWN || moveView.MoveButton == PSMButtonState_PRESSED) << 4);
						buttonsPressed |= ((moveView.PSButton == PSMButtonState_DOWN || moveView.PSButton == PSMButtonState_PRESSED) << 5);
						buttonsPressed |= ((moveView.StartButton == PSMButtonState_DOWN || moveView.StartButton == PSMButtonState_PRESSED) << 6);
						buttonsPressed |= ((moveView.SelectButton == PSMButtonState_DOWN || moveView.SelectButton == PSMButtonState_PRESSED) << 7);

						triggerValue = static_cast<float>(moveView.TriggerValue) / 255.f;
					}
					else if (controller_views[i]->ControllerType == PSMController_Virtual)
					{
						const PSMVirtualController &controllerView = controller_views[i]->ControllerState.VirtualController;

						triggerValue =
							(m_triggerAxisIndex >= 0 && m_triggerAxisIndex < controllerView.numAxes)
							? (float)controllerView.axisStates[m_triggerAxisIndex] / 255.f // axis value in range [0,255]
							: 0.f;

						int buttonCount = (controllerView.numButtons < 16) ? controllerView.numButtons : 16;
						for (int buttonIndex = 0; buttonIndex < buttonCount; ++buttonIndex)
						{
							int bit = (controllerView.buttonStates[buttonIndex] == PSMButtonState_DOWN || controllerView.buttonStates[buttonIndex] == PSMButtonState_PRESSED) ? 1 : 0;

							buttonsPressed |= (bit << buttonIndex);
						}
					}

					// HMD tracker? (always first ID)
					if (i == 0)
					{
						// Update tracking
						_trackingCallback(pose, _trackingCallbackContext);
					}
					else if (_controllerCallback != NULL)
					{
						// Update controller
						_controllerCallback(pose, buttonsPressed, triggerValue, _controllerCallbackContext);
					}
				}
			}
		}
	}
}

void IvryPsMoveClient::shutdown()
{
	log_msg("PSMoveClient is shutting down!\n");
	
	free_controller_views();
	free_hmd_views();

	// Close all active network connections
	PSM_Shutdown();
}

void IvryPsMoveClient::init_controller_views() 
{
	// Once created, updates will automatically get pushed into this view
	for (int i = 0; i < trackedControllerCount; i++)
	{
		PSM_AllocateControllerListener(trackedControllerIDs[i]);
		controller_views[i] = PSM_GetController(trackedControllerIDs[i]);

		// Kick off request to start streaming data from the first controller
		PSM_StartControllerDataStreamAsync(
			controller_views[i]->ControllerID, 
			m_sendSensorData ? PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData : PSMStreamFlags_includePositionData,
			&start_stream_request_ids[i]);

		//Set bulb color if specified
		if ((trackedBulbColors[i] >= 0) && (trackedBulbColors[i] < PSMTrackingColorType_MaxColorTypes)) {
			PSMRequestID request_id;
			PSM_SetControllerLEDColorAsync(controller_views[i]->ControllerID, trackedBulbColors[i], &request_id);
			PSM_EatResponse(request_id);
		}
	}
}

void IvryPsMoveClient::free_controller_views() 
{
	// Free any allocated controller views
	for (int i = 0; i < trackedControllerCount; i++)
	{
		if (controller_views[i] != nullptr)
		{
			// Stop the controller stream
			PSMRequestID request_id;
			PSM_StopControllerDataStreamAsync(controller_views[i]->ControllerID, &request_id);
			PSM_EatResponse(request_id);

			// Free out controller listener
			PSM_FreeControllerListener(controller_views[i]->ControllerID);
			controller_views[i] = nullptr;
		}
	}
}


void IvryPsMoveClient::init_hmd_views() 
{
	// Once created, updates will automatically get pushed into this view
	for (int i = 0; i < trackedHmdCount; i++)
	{
		PSM_AllocateHmdListener(trackedHmdIDs[i]);
		hmd_views[i] = PSM_GetHmd(trackedHmdIDs[i]);

		// Kick off request to start streaming data from the first hmd
		const bool bHasSensor= hmd_views[i]->HmdType == PSMHmd_Morpheus;
		PSM_StartHmdDataStreamAsync(
			hmd_views[i]->HmdID, 
			(bHasSensor && m_sendSensorData) 
			? PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData 
			: PSMStreamFlags_includePositionData,
			&start_stream_request_ids[i]);
	}
}

void IvryPsMoveClient::free_hmd_views() 
{
	// Free any allocated hmd views
	for (int i = 0; i < trackedHmdCount; i++)
	{
		if (hmd_views[i] != nullptr)
		{
			// Stop the hmd stream
			PSMRequestID request_id;
			PSM_StopHmdDataStreamAsync(hmd_views[i]->HmdID, &request_id);
			PSM_EatResponse(request_id);

			// Free out controller listener
			PSM_FreeHmdListener(hmd_views[i]->HmdID);
			hmd_views[i] = nullptr;
		}
	}
}

/** Log message **/
void IvryPsMoveClient::log_msg(const char *format, ...)
{
	va_list args;
	va_start(args, format);

	char msg[1024];
	vsprintf(msg, format, args);

	if (_logCallback)
	{
		// Send message back to driver for logging
		_logCallback(msg, _logCallbackContext);
	}
}
