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

/*
* Some portions edited from FreepieMoveClient.cpp in PSMoveFreepieBridge
* Some portions edited from ps_move_controller.cpp in PSMoveSteamVRBridge
*/

#include <tchar.h>
#include "IvryPsMoveTrackingApp.h"
#include "IvryPsMoveClient.h"

// Constants
#define MILLI_SECONDS_PER_SECOND (1000)
#define MICRO_SECONDS_PER_SECOND (1000*1000)
#define NANO_SECONDS_PER_SECOND (1000*1000*1000)

// Reset
#define PSMOVE_RESET_HOLD_DURATION 250 // ms

// Haptics
#define PSMOVE_HAPTICS_MAX_UPDATE_RATE 33 // ms - don't bother trying to update the rumble faster than 30fps (33ms)
#define PSMOVE_HAPTICS_MAX_PULSE_DURATION 5000 // us - docs suggest max pulse duration of 5ms, but we'll call 1ms max
#define PSMOVE_HAPTICS_DEFAULT_DURATION 0.f
#define PSMOVE_HAPTICS_DEFAULT_AMPLITUDE 1.f
#define PSMOVE_HAPTICS_DEFAULT_FREQUENCY 200.f

// Touchpad
#define PSMOVE_METRES_PER_TOUCHPAD_UNIT 0.075f
#define PSMOVE_MAX_TOUCHPAD_PRESS_TIME 2000 // ms

// Power
#define PSMOVE_POWER_UPDATE_INTERVAL 2000 // ms

# pragma comment(lib, "PSMoveClient_CAPI.lib")

/** SteamVR settings **/
#define IVRY_DRIVER_NAME "driver_ivry"
static const char * const k_pch_Ivry_Section = IVRY_DRIVER_NAME;
static const char * const k_pch_Ivry_k_pch_PsMoveUseGenericController_Bool = "psMoveUseGenericController";
static const char * const k_pch_Ivry_k_pch_PsMoveDisableHaptics_Bool = "psMoveDisableHaptics";
static const char * const k_pch_Ivry_k_pch_PsMoveDisableTouchpad_Bool = "psMoveDisableTouchpad";
static const char * const k_pch_Ivry_k_pch_PsMoveDelayTouchpad_Bool = "psMoveDelayTouchpad";

static inline vr::HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	vr::HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

IvryPsMoveClient::IvryPsMoveClient(IvryPsMoveTrackingApp *context)
	: _context(context)
	, _sensorTimer(INVALID_HANDLE_VALUE)
	, _sensorActive(FALSE)
	, _isOpen(false)
	, _trackingRate(PSMOVE_DEFAULT_TRACKING_RATE)
	, _trackedHmdCount(0)
	, _trackedControllerCount(0)
{
	_deviceType = PSM_DeviceTypeUnkown;
	_deviceId = PSM_ControllerId_None;

	_leftControllerId = _rightControllerId = PSM_ControllerId_None;

	memset(_trackedHmdIDs, 0, sizeof(_trackedHmdIDs));
	memset(_trackedControllerIDs, 0, sizeof(_trackedControllerIDs));
	memset(_trackedBulbColors, PSMTrackingColorType_MaxColorTypes, sizeof(_trackedBulbColors));

	_sendSensorData = true;

	memset(_controllerState, 0, sizeof(_controllerState));
	for (int i = 0; i < PSMControllerIndex_Count; i++)
	{
		_controllerState[i].hapticState.pendingDurationSecs = PSMOVE_HAPTICS_DEFAULT_DURATION;
		_controllerState[i].hapticState.pendingAmplitude = PSMOVE_HAPTICS_DEFAULT_AMPLITUDE;
		_controllerState[i].hapticState.pendingFrequency = PSMOVE_HAPTICS_DEFAULT_FREQUENCY;
		_controllerState[i].hapticState.lastRumbleSentTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch());
		_controllerState[i].hapticState.lastRumbleSentValid = false;
	}

	memset(_controllerTouchpadMapping, 0, sizeof(_controllerTouchpadMapping));
	_controllerTouchpadMapping[PSMOVE_BUTTON_MOVE] = PSMOVE_TRACKPAD_ACTION_PRESS;

	for (int i = 0; i < PSMOVE_MAX_CONTROLLERS; i++)
	{
		_hmdViews[i] = nullptr;
		_controllerViews[i] = nullptr;
		_startStreamRequestIds[i] = -1;
	}

	// Read settings
	load_settings();
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
	_sensorActive = enable;

	return true;
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
			LARGE_INTEGER dueTime;
			dueTime.QuadPart = (LONGLONG)((INT64_C(-100) * (int64_t)NANO_SECONDS_PER_SECOND) / _trackingRate);
			::SetWaitableTimer(_sensorTimer, &dueTime, 0, NULL, NULL, FALSE);
			::WaitForSingleObject(_sensorTimer, (DWORD)(MILLI_SECONDS_PER_SECOND / _trackingRate));

			// Update tracking
			update();
		}

		::CloseHandle(_sensorTimer);
		_sensorTimer = INVALID_HANDLE_VALUE;
	}
	else
	{
		log_msg("ERROR: Could not create PS Move sensor polling timer (%08x)!\n", ::GetLastError());
	}
}

/** (Re)load tracker settings **/
void IvryPsMoveClient::load_settings()
{
	_psMoveUseNativeDriver = true;
	_psMoveHapticsEnabled = true;
	_psMoveEmulateTouchpad = true;
	_psMoveTouchpadDelay = false;
}

void IvryPsMoveClient::handle_client_psmove_response(const PSMMessage *message)
{
	switch (message->response_data.payload_type) 
	{
	case PSMResponseMessage::_responsePayloadType_HmdList:
		handle_hmd_list_response(&message->response_data.payload.hmd_list, message->response_data.opaque_request_handle);
		break;
	case PSMResponseMessage::_responsePayloadType_ControllerList:
		handle_controller_list_response(&message->response_data.payload.controller_list, message->response_data.opaque_request_handle);
		break;
	default:
		break;
	}
}

void IvryPsMoveClient::handle_connect_to_service()
{
	// Get protocol version
	PSMRequestID request_id;
	PSM_GetServiceVersionStringAsync(&request_id);
	PSM_RegisterCallback(request_id, handle_service_version_response, this);
}

void IvryPsMoveClient::handle_service_version_response(const PSMResponseMessage *response, void *userdata)
{
	IvryPsMoveClient *thiz = reinterpret_cast<IvryPsMoveClient*>(userdata);
	if (thiz != NULL)
	{
		switch (response->result_code)
		{
			case PSMResult::PSMResult_Success:
				{
					const std::string service_version = response->payload.service_version.version_string;
					const std::string local_version = PSM_GetClientVersionString();

					// Protocol version matches?
					if (service_version == local_version)
					{
						// Get headset and controller lists
						PSM_GetHmdListAsync(nullptr);
						PSM_GetControllerListAsync(nullptr);
					}
					else
					{
						thiz->log_msg("Protocol version mismatch! Expected '%s', got '%s'\n", local_version.c_str(), service_version.c_str());
						thiz->close();
					}
				}
				break;

			case PSMResult::PSMResult_Error:
			case PSMResult::PSMResult_Canceled:
				break;
		}
	}
}

void IvryPsMoveClient::handle_hmd_list_response(const PSMHmdList *hmd_list, const PSMResponseHandle response_handle)
{
	log_msg("Found %d virtual headset(s)\n", hmd_list->count);

	free_hmd_views();

	if (hmd_list->count > 0)
	{
		_trackedHmdIDs[0] = hmd_list->hmd_id[0];
		_deviceType = PSM_DeviceTypeHMD;
		_deviceId = hmd_list->hmd_id[0];

		// Initialise HMD tracker
		_trackedHmdCount = 1;
		init_hmd_views();
	}
	else
	{
		_trackedHmdCount = 0;
	}
}

void IvryPsMoveClient::handle_controller_list_response(const PSMControllerList *controller_list, const PSMResponseHandle response_handle)
{
	log_msg("Found %d controller(s)\n", controller_list->count);

	free_controller_views();

	// Look for controller specified as HMD tracker?
	bool found = (_deviceType == PSM_DeviceTypeHMD);
	if (!found && _controllerSerialNoFilter.length() > 0)
	{
		for (int i = 0; i < controller_list->count; i++)
		{
			if (_controllerSerialNoFilter.compare(controller_list->controller_serial[i]) == 0)
			{
				log_msg("Using specified controller id:%d (%s) as HMD tracker\n",
					controller_list->controller_id[i], controller_list->controller_serial[i]);
				_deviceId = controller_list->controller_id[i];
				_deviceType = PSM_DeviceTypeController;
				found = true;
				break;
			}
		}
	}

	// No HMD tracker and more than 2 controllers?
	if (!found && controller_list->count > 2)
	{
		log_msg("Using controller id:%d (%s) as HMD tracker\n",
			controller_list->controller_id[2], controller_list->controller_serial[2]);
		_controllerSerialNoFilter = controller_list->controller_serial[2];
		_deviceId = controller_list->controller_id[2];
		_deviceType = PSM_DeviceTypeController;
	}

	// Any controllers attached?
	if (controller_list->count > 0)
	{
		PSMControllerID leftId = PSM_ControllerId_None, rightId = PSM_ControllerId_None;
		for (int i = 0; i < controller_list->count; i++)
		{
			_trackedControllerIDs[i] = controller_list->controller_id[i];

			// Skip HMD tracker controller
			if (_deviceType == PSM_DeviceTypeController
				&& controller_list->controller_id[i] == _deviceId)
			{
				continue;
			}

			// Left controller?
			if ((leftId == PSM_ControllerId_None && controller_list->controller_hand[i] == PSMControllerHand_Left)
				|| controller_list->controller_id[i] == _leftControllerId)
			{
				leftId = controller_list->controller_id[i];
			}
			// Right controller?
			if ((rightId == PSM_ControllerId_None && controller_list->controller_hand[i] == PSMControllerHand_Right)
				|| controller_list->controller_id[i] == _rightControllerId)
			{
				rightId = controller_list->controller_id[i];
			}
		}

		// No right controller?
		if (rightId == PSM_ControllerId_None)
		{
			// Find first "any hand" controller
			for (int i = 0; i < controller_list->count; i++)
			{
				// Skip HMD tracker controller
				if (_deviceType == PSM_DeviceTypeController
					&& controller_list->controller_id[i] == _deviceId)
				{
					continue;
				}

				// Found controller?
				if (controller_list->controller_hand[i] == PSMControllerHand_Any)
				{
					rightId = controller_list->controller_id[i];
					break;
				}
			}
		}

		// No left controller?
		if (leftId == PSM_ControllerId_None)
		{
			// Find next "any hand" controller
			for (int i = 0; i < controller_list->count; i++)
			{
				// Skip HMD tracker controller
				if (_deviceType == PSM_DeviceTypeController
					&& controller_list->controller_id[i] == _deviceId)
				{
					continue;
				}

				// Found controller?
				if (controller_list->controller_hand[i] == PSMControllerHand_Any 
					&& controller_list->controller_id[i] != rightId)
				{
					leftId = controller_list->controller_id[i];
					break;
				}
			}
		}

		// Left controller changed?
		if (leftId != _leftControllerId)
		{
			// Controller added?
			if (_leftControllerId == PSM_ControllerId_None)
			{
				_leftControllerId = leftId;
				_context->ControllerConnected(leftId, IVRY_TRACKING_HAND_LEFT, _psMoveUseNativeDriver ? "PSMove" : "Generic");
			}
			// Controller removed?
			else if (leftId == PSM_ControllerId_None)
			{
				_context->ControllerRemoved(_leftControllerId);
				_leftControllerId = PSM_ControllerId_None;
			}
		}
		// Right controller changed?
		if (rightId != _rightControllerId)
		{
			// Controller added?
			if (_rightControllerId == PSM_ControllerId_None)
			{
				_rightControllerId = rightId;
				_context->ControllerConnected(rightId, IVRY_TRACKING_HAND_RIGHT, _psMoveUseNativeDriver ? "PSMove" : "Generic");
			}
			// Controller removed?
			else if (rightId == PSM_ControllerId_None)
			{
				_context->ControllerRemoved(_rightControllerId);
				_rightControllerId = PSM_ControllerId_None;
			}
		}

		// Initialise controllers
		_trackedControllerCount = controller_list->count;
		init_controller_views();
	}
	else
	{
		_trackedControllerCount = 0;
	}
}

void IvryPsMoveClient::handle_client_psmove_event(const PSMMessage *message)
{
	switch (message->event_data.event_type)
	{
	case PSMEventMessage::PSMEvent_connectedToService:
		//log_msg("Connected to service\n");
		handle_connect_to_service();
		break;
	case PSMEventMessage::PSMEvent_failedToConnectToService:
		log_msg("Failed to connect to service\n");
		break;
	case PSMEventMessage::PSMEvent_disconnectedFromService:
		//log_msg("Disconnected from service\n");
		break;
	case PSMEventMessage::PSMEvent_hmdListUpdated:
		PSM_GetHmdListAsync(nullptr);
		break;
	case PSMEventMessage::PSMEvent_controllerListUpdated:
		PSM_GetControllerListAsync(nullptr);
		break;
	case PSMEventMessage::PSMEvent_trackerListUpdated:
		//log_msg("Tracker list updated. Ignored.\n");
		break;
	case PSMEventMessage::PSMEvent_systemButtonPressed:
		//log_msg("System button pressed. Ignored.\n");
		break;
	case PSMEventMessage::PSMEvent_opaqueServiceEvent:
		//log_msg("Opaque service event(%d). Ignored.\n", static_cast<int>(message->event_data.event_type));
		break;
	default:
		//log_msg("unhandled event(%d)\n", static_cast<int>(message->event_data.event_type));
		break;
	}
}

bool IvryPsMoveClient::startup()
{
	bool success = _context != NULL && _context->IsPsMoveServiceRunning();

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
		_lastReportFpsTimestamp =
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
			handle_client_psmove_response(&message);
			break;
		case PSMMessage::_messagePayloadType_Event:
			handle_client_psmove_event(&message);
			break;
		}
	}

	// Update tracking?
	if (_context != NULL)
	{
		if (_deviceType == PSM_DeviceTypeHMD)
		{
			for (int i = 0; i < _trackedHmdCount; i++)
			{
				if (_hmdViews[i] && _hmdViews[i]->bValid)
				{
					std::chrono::milliseconds now =
						std::chrono::duration_cast<std::chrono::milliseconds>(
							std::chrono::system_clock::now().time_since_epoch());
					std::chrono::milliseconds diff = now - _lastReportFpsTimestamp;

					PSMPosef hmdPose;
					if (PSM_GetHmdPose(_hmdViews[i]->HmdID, &hmdPose) == PSMResult_Success)
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

						if (_hmdViews[i]->HmdType == PSMHmd_Morpheus)
						{
							PSMQuatf normalizedQuat = PSM_QuatfNormalizeWithDefault(&hmdPose.Orientation, k_psm_quaternion_identity);
							pose.qRotation.w = normalizedQuat.w;
							pose.qRotation.x = normalizedQuat.x;
							pose.qRotation.y = normalizedQuat.y;
							pose.qRotation.z = normalizedQuat.z;

							if (_sendSensorData)
							{
								PSMMorpheusCalibratedSensorData sensors = _hmdViews[i]->HmdState.MorpheusState.CalibratedSensorData;

								pose.vecAcceleration[0] = sensors.Accelerometer.x;
								pose.vecAcceleration[1] = sensors.Accelerometer.y;
								pose.vecAcceleration[2] = sensors.Accelerometer.z;

								pose.vecAngularAcceleration[0] = sensors.Gyroscope.x;
								pose.vecAngularAcceleration[1] = sensors.Gyroscope.y;
								pose.vecAngularAcceleration[2] = sensors.Gyroscope.z;
							}
						}

						// HMD tracker?
						if (_hmdViews[i]->HmdID == _deviceId)
						{
							// Update tracking
							_context->PoseUpdated(pose);
						}
					}
				}
			}
		}

		// Update PS Move controllers
		update_controllers();
	}
}

static uint32_t _getButtonMask(const PSMButtonState &state, PSMButtonState type, PSMove_Button button)
{
	if (state == type) return button;
	else return 0;
}

static uint32_t _getButtonsMask(const PSMPSMove &moveView, PSMButtonState type)
{
	uint32_t buttons = 0;

	// Get button states
	buttons |= _getButtonMask(moveView.SquareButton, type, Btn_SQUARE);
	buttons |= _getButtonMask(moveView.TriangleButton, type, Btn_TRIANGLE);
	buttons |= _getButtonMask(moveView.CrossButton, type, Btn_CROSS);
	buttons |= _getButtonMask(moveView.CircleButton, type, Btn_CIRCLE);
	buttons |= _getButtonMask(moveView.MoveButton, type, Btn_MOVE);
	buttons |= _getButtonMask(moveView.PSButton, type, Btn_PS);
	buttons |= _getButtonMask(moveView.StartButton, type, Btn_START);
	buttons |= _getButtonMask(moveView.SelectButton, type, Btn_SELECT);
	buttons |= _getButtonMask(moveView.TriggerButton, type, Btn_T);

	return buttons;
}

void IvryPsMoveClient::update_controllers()
{
	for (int i = 0; i < _trackedControllerCount; i++)
	{
		// PS Move controller?
		if (_controllerViews[i] && _controllerViews[i]->bValid &&
			_controllerViews[i]->ControllerType == PSMController_Move)
		{
			std::chrono::milliseconds now =
				std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::system_clock::now().time_since_epoch());
			std::chrono::milliseconds diff = now - _lastReportFpsTimestamp;

			vr::DriverPose_t pose;
			memset(&pose, 0, sizeof(pose));
			pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
			pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
			pose.poseIsValid = true;
			pose.result = vr::TrackingResult_Running_OK;
			pose.deviceIsConnected = true;

			PSMPosef controllerPose;
			PSM_GetControllerPose(_controllerViews[i]->ControllerID, &controllerPose);

			pose.vecPosition[0] = controllerPose.Position.x * PSMOVE_SCALE_FACTOR;
			pose.vecPosition[1] = controllerPose.Position.y * PSMOVE_SCALE_FACTOR;
			pose.vecPosition[2] = controllerPose.Position.z * PSMOVE_SCALE_FACTOR;

			PSMQuatf normalizedQuat = PSM_QuatfNormalizeWithDefault(&controllerPose.Orientation, k_psm_quaternion_identity);
			pose.qRotation.w = normalizedQuat.w;
			pose.qRotation.x = normalizedQuat.x;
			pose.qRotation.y = normalizedQuat.y;
			pose.qRotation.z = normalizedQuat.z;

			if (_sendSensorData)
			{
				const PSMPSMove &moveView = _controllerViews[i]->ControllerState.PSMoveState;
				const PSMPSMoveCalibratedSensorData &sensors = moveView.CalibratedSensorData;

				pose.vecAcceleration[0] = sensors.Accelerometer.x;
				pose.vecAcceleration[1] = sensors.Accelerometer.y;
				pose.vecAcceleration[2] = sensors.Accelerometer.z;

				pose.vecAngularAcceleration[0] = sensors.Gyroscope.x;
				pose.vecAngularAcceleration[1] = sensors.Gyroscope.y;
				pose.vecAngularAcceleration[2] = sensors.Gyroscope.z;
			}

			// HMD tracker?
			if (_deviceType == PSM_DeviceTypeController && _controllerViews[i]->ControllerID == _deviceId)
			{
				// Update tracking
				_context->PoseUpdated(pose);
			}
			else
			{
				// Update controller
				int index = _controllerViews[i]->ControllerID == _leftControllerId ? PSMControllerIndex_Left
					: (_controllerViews[i]->ControllerID == _rightControllerId ? PSMControllerIndex_Right : PSMControllerIndex_Count);
				if (index < PSMControllerIndex_Count)
				{
					IvryPsMoveTrackingPsMoveState *state = &_controllerState[index];
					const PSMPSMove &moveView = _controllerViews[i]->ControllerState.PSMoveState;

					// Set controller pose
					state->pose = pose;

					// Get button states
					uint32_t buttons = _getButtonsMask(moveView, PSMButtonState_DOWN);
					uint32_t pressed = _getButtonsMask(moveView, PSMButtonState_PRESSED);
					uint32_t released = _getButtonsMask(moveView, PSMButtonState_RELEASED);

					// Trackpad
					state->touchpad.isPressed = state->touchpad.isTouched = (buttons & Btn_MOVE) != 0;

					// Buttons
					state->buttonA.isTouched = state->buttonA.isPressed = (buttons & Btn_CROSS) != 0;
					state->buttonA.scalar.value = state->buttonA.isPressed ? 1.0f : 0.0f;
					state->buttonB.isTouched = state->buttonB.isPressed = (buttons & Btn_CIRCLE) != 0;
					state->buttonB.scalar.value = state->buttonB.isPressed ? 1.0f : 0.0f;
					state->buttonX.isTouched = state->buttonX.isPressed = (buttons & Btn_SQUARE) != 0;
					state->buttonX.scalar.value = state->buttonX.isPressed ? 1.0f : 0.0f;
					state->buttonY.isTouched = state->buttonY.isPressed = (buttons & Btn_TRIANGLE) != 0;
					state->buttonY.scalar.value = state->buttonY.isPressed ? 1.0f : 0.0f;

					// Grip/trigger
					state->grip.isTouched = state->grip.isPressed = (buttons & Btn_SELECT) != 0;
					state->grip.scalar.value = state->grip.isPressed ? 1.0f : 0.0f;
					state->trigger.isTouched = state->trigger.isPressed = (buttons & Btn_T) != 0;
					state->trigger.scalar.value = state->trigger.isPressed ? moveView.TriggerValue / 255.0f : 0.0f;

					// Back/enter
					state->back.isPressed = state->back.isTouched = (buttons & Btn_START) != 0;
					state->enter.isTouched = state->enter.isPressed = (buttons & Btn_PS) != 0;

					// Update button states
					int buttonMask[PSMOVE_BUTTON_COUNT] = { Btn_PS, Btn_START, Btn_SELECT, Btn_MOVE, Btn_CROSS, Btn_CIRCLE, Btn_SQUARE, Btn_TRIANGLE };
					for (int button = 0; button < PSMOVE_BUTTON_COUNT; button++)
					{
						if ((released & buttonMask[button]) != 0)
						{
							state->buttonState[button] = PSMButtonState_RELEASED;
						}
						else if ((pressed & buttonMask[button]) != 0)
						{
							state->buttonState[button] = PSMButtonState_PRESSED;
						}
						else if ((buttons & buttonMask[button]) != 0)
						{
							state->buttonState[button] = PSMButtonState_DOWN;
						}
						else
						{
							state->buttonState[button] = PSMButtonState_UP;
						}
					}

					// Recentering
					PSMButtonState resetPoseButtonState = state->buttonState[index == PSMControllerIndex_Right ? PSMOVE_BUTTON_START : PSMOVE_BUTTON_SELECT];
					switch (resetPoseButtonState)
					{
					case PSMButtonState_PRESSED:
						state->resetState.pressTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
							std::chrono::system_clock::now().time_since_epoch());
						break;

					case PSMButtonState_DOWN:
						// Didn't recenter yet?
						if (!state->resetState.reset)
						{
							// Held?
							if ((std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch()) - state->resetState.pressTimestamp).count() >= PSMOVE_RESET_HOLD_DURATION)
							{
								/*// Not using Monado orientation?
								if (!_psMoveUseMonadoOrientation)
								{
									// Reset controller orientation
									psmove_reset_orientation(controller->move);
								}*/

								state->resetState.reset = true;
							}
						}
						break;

					case PSMButtonState_RELEASED:
						state->resetState.reset = false;
						break;

					default:
						// Do nothing
						break;
					}

					// Emulating touchpad?
					if (_psMoveEmulateTouchpad)
					{
						// Update emulated touchpad
						controller_touchpad_update(index);
					}

					// Update controller
					_context->ControllerUpdated(_controllerViews[i]->ControllerID, state);

					// Update controller haptics
					controller_haptic_update(_controllerViews[i]->ControllerID, state);

					// Update controller power state
					state->powerState.state = moveView.BatteryValue;
					controller_power_update(_controllerViews[i]->ControllerID, state);
				}
			}
		}
	}
}

/*
	Emulate trackpad (from PSMoveSteamVRBridge ps_move_controller.cpp)

	In a nutshell, upon the move button being pressed the initial pose is captured and rotated relative to the
	controller's position. After a buttonheld threshold it's considered held and the next controller pose is captured
	and again rotated. The initial and current are subtracted to get the distance in meters between the two. The rotation
	is important since it must be relative to the controller not the world. After the rotation a repeatable calculation of
	distance between the two on the z and x axis can be determined. This is then scaled and applied to the x and y axis
	of the trackpad. When the ps move button is no longer pressed the trackpad axis is reset to 0,0 and past state is
	cleared.

	```
	Initial origin pose:

		z   _
		|  (_)
		|  {0} <- Move button pressed and held facing forward on the y axis
		|  |*|
		|  {_}
		|_________ x
	   /
	  /
	 /
	y


	Future pose update:

		z                 _
		|       7.5cm    (_)
		|     ------->   {0} <- Move button still held facing forward on the x axis
		|      moved     |*|
		|      right     {_}
		|_________ x
	   /
	  /
	 /
	y
	```
*/

/** OpenVR math utility functions **/
static vr::HmdQuaternion_t _quaternionConjugate(const vr::HmdQuaternion_t &q)
{
	return { q.w, -q.x, -q.y, -q.z };
}

static void _vectorRotate(double(&result)[3], const vr::HmdQuaternion_t &q, const double(&v)[3])
{
	result[0] = q.w*q.w*v[0] + 2 * q.y*q.w*v[2] - 2 * q.z*q.w*v[1] + q.x*q.x*v[0] + 2 * q.y*q.x*v[1] + 2 * q.z*q.x*v[2] - q.z*q.z*v[0] - q.y*q.y*v[0];
	result[1] = 2 * q.x*q.y*v[0] + q.y*q.y*v[1] + 2 * q.z*q.y*v[2] + 2 * q.w*q.z*v[0] - q.z*q.z*v[1] + q.w*q.w*v[1] - 2 * q.x*q.w*v[2] - q.x*q.x*v[1];
	result[2] = 2 * q.x*q.z*v[0] + 2 * q.y*q.z*v[1] + q.z*q.z*v[2] - 2 * q.w*q.y*v[0] - q.y*q.y*v[2] + 2 * q.w*q.x*v[1] - q.x*q.x*v[2] + q.w*q.w*v[2];
}

static void _getPositionInRotationSpace(double(&out_position)[3], const vr::HmdQuaternion_t rotation, const double(&position)[3])
{
	vr::HmdQuaternion_t viewOrientationInverse = _quaternionConjugate(rotation);

	_vectorRotate(out_position, viewOrientationInverse, position);
}

static void _vectorSubtract(double(&out)[3], const double(&a)[3], const double(&b)[3])
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

// Updates the state of the controllers touchpad axis relative to its position over time and active state.
void IvryPsMoveClient::controller_touchpad_update(int index)
{
	if (index < PSMControllerIndex_Count)
	{
		IvryPsMoveTrackingPsMoveState *state = &_controllerState[index];

		// Find the highest priority emulated touch pad action (if any)
		IvryPsMoveTrackingPsMoveTouchpadAction highestPriorityAction = PSMOVE_TRACKPAD_ACTION_NONE;
		for (int buttonIndex = 0; buttonIndex < static_cast<int>(PSMOVE_BUTTON_COUNT); ++buttonIndex)
		{
			IvryPsMoveTrackingPsMoveTouchpadAction action = _controllerTouchpadMapping[buttonIndex];
			if (action != PSMOVE_TRACKPAD_ACTION_NONE)
			{
				PSMButtonState button_state = state->buttonState[buttonIndex];
				if (button_state == PSMButtonState_DOWN || button_state == PSMButtonState_PRESSED)
				{
					if (action >= highestPriorityAction)
					{
						highestPriorityAction = action;
					}

					if (action >= PSMOVE_TRACKPAD_ACTION_PRESS)
					{
						break;
					}
				}
			}
		}

		float touchpad_x = 0.f;
		float touchpad_y = 0.f;
		PSMButtonState touchPadTouchedState = PSMButtonState_UP;
		PSMButtonState touchPadPressedState = PSMButtonState_UP;

		if (highestPriorityAction == PSMOVE_TRACKPAD_ACTION_TOUCH)
		{
			touchPadTouchedState = PSMButtonState_DOWN;
		}
		else if (highestPriorityAction == PSMOVE_TRACKPAD_ACTION_PRESS)
		{
			touchPadTouchedState = PSMButtonState_DOWN;
			touchPadPressedState = PSMButtonState_DOWN;
		}

		// If the action specifies a specific trackpad direction,
		// then use the given trackpad axis
		if (highestPriorityAction > PSMOVE_TRACKPAD_ACTION_PRESS)
		{
			touchPadTouchedState = PSMButtonState_DOWN;
			touchPadPressedState = PSMButtonState_DOWN;

			switch (highestPriorityAction)
			{
			case PSMOVE_TRACKPAD_ACTION_LEFT:
				touchpad_x = -1.f;
				touchpad_y = 0.f;
				break;
			case PSMOVE_TRACKPAD_ACTION_UP:
				touchpad_x = 0.f;
				touchpad_y = 1.f;
				break;
			case PSMOVE_TRACKPAD_ACTION_RIGHT:
				touchpad_x = 1.f;
				touchpad_y = 0.f;
				break;
			case PSMOVE_TRACKPAD_ACTION_DOWN:
				touchpad_x = 0.f;
				touchpad_y = -1.f;
				break;
			case PSMOVE_TRACKPAD_ACTION_UPLEFT:
				touchpad_x = -0.707f;
				touchpad_y = 0.707f;
				break;
			case PSMOVE_TRACKPAD_ACTION_UPRIGHT:
				touchpad_x = 0.707f;
				touchpad_y = 0.707f;
				break;
			case PSMOVE_TRACKPAD_ACTION_DOWNLEFT:
				touchpad_x = -0.707f;
				touchpad_y = -0.707f;
				break;
			case PSMOVE_TRACKPAD_ACTION_DOWNRIGHT:
				touchpad_x = 0.707f;
				touchpad_y = -0.707f;
				break;
			}
		}

		// Otherwise if the action was just a touch or press,
		// then use spatial offset method for determining touchpad axis
		if (highestPriorityAction == PSMOVE_TRACKPAD_ACTION_TOUCH || highestPriorityAction == PSMOVE_TRACKPAD_ACTION_PRESS)
		{
			bool updateTouchpad = true;

			if (_psMoveTouchpadDelay)
			{
				std::chrono::milliseconds now =
					std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::system_clock::now().time_since_epoch());

				if (!state->touchpadState.wasActive)
				{
					// true if the touchpad has been active for more than the max time required to hold it
					updateTouchpad = (now - state->touchpadState.pressTimestamp).count() >= PSMOVE_MAX_TOUCHPAD_PRESS_TIME;
				}

				state->touchpadState.pressTimestamp = now;
			}

			if (updateTouchpad)
			{
				if (!state->touchpadState.wasActive)
				{
					// Pressed
					state->touchpadState.orientationAtPress = state->pose.qRotation;
					_getPositionInRotationSpace(state->touchpadState.positionAtPress, state->touchpadState.orientationAtPress, state->pose.vecPosition);
				}
				else
				{
					// Held
					double position[3], offset[3];
					_getPositionInRotationSpace(position, state->touchpadState.orientationAtPress, state->pose.vecPosition);
					_vectorSubtract(offset, position, state->touchpadState.positionAtPress);

					touchpad_x = fminf(fmaxf((float)(offset[0] / PSMOVE_METRES_PER_TOUCHPAD_UNIT), -1.0f), 1.0f);
					touchpad_y = fminf(fmaxf((float)(-offset[2] / PSMOVE_METRES_PER_TOUCHPAD_UNIT), -1.0f), 1.0f);
				}
			}
		}

		state->touchpad.isTouched = touchPadTouchedState;
		state->touchpad.isPressed = touchPadPressedState;

		state->touchpad.axis.x = touchpad_x;
		state->touchpad.axis.y = touchpad_y;

		// Remember if the touchpad was active the previous frame for edge detection
		state->touchpadState.wasActive = highestPriorityAction != PSMOVE_TRACKPAD_ACTION_NONE;
	}
}

void IvryPsMoveClient::controller_haptics(uint32_t id, float fDurationSeconds, float fFrequency, float fAmplitude)
{
	for (int i = 0; i < _trackedControllerCount; i++)
	{
		if (_controllerViews[i] && _controllerViews[i]->bValid
			&& _controllerViews[i]->ControllerType == PSMController_Move
			&& _controllerViews[i]->ControllerID == id)
		{
			int index = _controllerViews[i]->ControllerID == _leftControllerId ? PSMControllerIndex_Left
				: (_controllerViews[i]->ControllerID == _rightControllerId ? PSMControllerIndex_Right : PSMControllerIndex_Count);
			if (index < PSMControllerIndex_Count)
			{
				IvryPsMoveTrackingPsMoveState *state = &_controllerState[index];

				state->hapticState.pendingDurationSecs = fDurationSeconds;
				state->hapticState.pendingFrequency = fFrequency;
				state->hapticState.pendingAmplitude = fAmplitude;

				return;
			}
		}
	}
}

void IvryPsMoveClient::controller_haptic_update(PSMControllerID id, IvryPsMoveTrackingPsMoveState *state)
{
	if (state != NULL)
	{
		if (_psMoveHapticsEnabled)
		{
			// pulse duration - the length of each pulse
			// amplitude - strength of vibration
			// frequency - speed of each pulse

			// convert to microseconds, the max duration received from OpenVR appears to be 5 micro seconds
			uint16_t pendingHapticPulseDurationMicroSecs =
				static_cast<uint16_t>(state->hapticState.pendingDurationSecs * MICRO_SECONDS_PER_SECOND);

			std::chrono::milliseconds now =
				std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::system_clock::now().time_since_epoch());
			bool timeoutElapsed = true;

			if (state->hapticState.lastRumbleSentValid)
			{
				timeoutElapsed = ((now - state->hapticState.lastRumbleSentTimestamp).count() >= PSMOVE_HAPTICS_MAX_UPDATE_RATE);
			}

			// See if a rumble request hasn't come too recently
			if (timeoutElapsed)
			{
				float rumble_fraction =
					(static_cast<float>(pendingHapticPulseDurationMicroSecs) / PSMOVE_HAPTICS_MAX_PULSE_DURATION)
					* state->hapticState.pendingAmplitude;

				// Unless a zero rumble intensity was explicitly set, 
				// don't rumble less than 35% (no enough to feel)
				if (state->hapticState.pendingDurationSecs != 0)
				{
					if (rumble_fraction < 0.35f)
					{
						// rumble values less 35% isn't noticeable
						rumble_fraction = 0.35f;
					}
				}

				// Keep the pulse intensity within reasonable bounds
				if (rumble_fraction > 1.f)
				{
					rumble_fraction = 1.f;
				}

				// Actually set controller rumble (will be sent to controller on next update)
				PSM_SetControllerRumble(id, PSMControllerRumbleChannel_All, rumble_fraction);

				// Remember the last rumble we set and when we set it
				state->hapticState.lastRumbleSentTimestamp = now;
				state->hapticState.lastRumbleSentValid = true;

				// Reset the pending haptic pulse duration.
				// If another call to ControllerHaptics() is made later, it will stomp this value.
				// If no future haptic event is received by ServerDriver then the next call to PsMoveHapticUpdate()
				// in PSMOVE_HAPTICS_MAX_UPDATE_RATE milliseconds will set the rumble_fraction to 0.f
				// This effectively makes the shortest rumble pulse PSMOVE_HAPTICS_MAX_UPDATE_RATE milliseconds.
				state->hapticState.pendingDurationSecs = PSMOVE_HAPTICS_DEFAULT_DURATION;
				state->hapticState.pendingAmplitude = PSMOVE_HAPTICS_DEFAULT_AMPLITUDE;
				state->hapticState.pendingFrequency = PSMOVE_HAPTICS_DEFAULT_FREQUENCY;
			}
		}
		else
		{
			// Reset the pending haptic pulse duration since rumble is suppressed.
			state->hapticState.pendingDurationSecs = PSMOVE_HAPTICS_DEFAULT_DURATION;
			state->hapticState.pendingAmplitude = PSMOVE_HAPTICS_DEFAULT_AMPLITUDE;
			state->hapticState.pendingFrequency = PSMOVE_HAPTICS_DEFAULT_FREQUENCY;
		}
	}
}

void IvryPsMoveClient::controller_power_update(PSMControllerID id, IvryPsMoveTrackingPsMoveState *state)
{
	if (state != NULL)
	{
		std::chrono::milliseconds now =
			std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::system_clock::now().time_since_epoch());

		// Power
		if ((now - state->powerState.lastStatusSentTimestamp).count() >= PSMOVE_POWER_UPDATE_INTERVAL)
		{
			IvryTrackingPowerState powerState = IVRY_TRACKING_POWER_UNPLUGGED;
			float level = 0;

			switch (state->powerState.state)
			{
			case PSMBattery_0:
				level = 0;
				break;

			case PSMBattery_20:
				level = 0.2f;
				break;

			case PSMBattery_40:
				level = 0.4f;
				break;

			case PSMBattery_60:
				level = 0.6f;
				break;

			case PSMBattery_80:
				level = 0.8f;
				break;

			case PSMBattery_100:
				level = 1.0f;
				break;

			case PSMBattery_Charging:
				level = 0.99f;
				powerState = IVRY_TRACKING_POWER_CHARGING;
				break;

			case PSMBattery_Charged:
				level = 1.0f;
				powerState = IVRY_TRACKING_POWER_CHARGING;
				break;

			default:
				powerState = IVRY_TRACKING_POWER_UNKNOWN;
				break;
			}

			// Send power update to driver
			_context->ControllerPowerUpdated(id, powerState, level);
			state->powerState.lastStatusSentTimestamp = now;
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
	for (int i = 0; i < _trackedControllerCount; i++)
	{
		PSM_AllocateControllerListener(_trackedControllerIDs[i]);
		_controllerViews[i] = PSM_GetController(_trackedControllerIDs[i]);

		// Kick off request to start streaming data from the first controller
		unsigned int flags = PSMStreamFlags_includePositionData;
		if (_sendSensorData) flags |= PSMStreamFlags_includePhysicsData | PSMStreamFlags_includeCalibratedSensorData;
		PSM_StartControllerDataStreamAsync(
			_controllerViews[i]->ControllerID, 
			flags,
			&_startStreamRequestIds[i]);

		// Set bulb color if specified
		if ((_trackedBulbColors[i] >= 0) && (_trackedBulbColors[i] < PSMTrackingColorType_MaxColorTypes)) 
		{
			PSMRequestID request_id;
			PSM_SetControllerLEDColorAsync(_controllerViews[i]->ControllerID, _trackedBulbColors[i], &request_id);
			PSM_EatResponse(request_id);
		}
	}
}

void IvryPsMoveClient::free_controller_views() 
{
	// Free any allocated controller views
	for (int i = 0; i < _trackedControllerCount; i++)
	{
		if (_controllerViews[i] != nullptr)
		{
			// Stop the controller stream
			PSMRequestID request_id;
			PSM_StopControllerDataStreamAsync(_controllerViews[i]->ControllerID, &request_id);
			PSM_EatResponse(request_id);

			// Free out controller listener
			PSM_FreeControllerListener(_controllerViews[i]->ControllerID);
			_controllerViews[i] = nullptr;
		}
	}
}

void IvryPsMoveClient::init_hmd_views() 
{
	// Once created, updates will automatically get pushed into this view
	for (int i = 0; i < _trackedHmdCount; i++)
	{
		PSM_AllocateHmdListener(_trackedHmdIDs[i]);
		_hmdViews[i] = PSM_GetHmd(_trackedHmdIDs[i]);

		// Kick off request to start streaming data from the first hmd
		const bool bHasSensor = _hmdViews[i]->HmdType == PSMHmd_Morpheus;
		PSM_StartHmdDataStreamAsync(
			_hmdViews[i]->HmdID, 
			(bHasSensor && _sendSensorData) 
			? PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData
			: PSMStreamFlags_includePositionData,
			&_startStreamRequestIds[i]);
	}
}

void IvryPsMoveClient::free_hmd_views() 
{
	// Free any allocated hmd views
	for (int i = 0; i < _trackedHmdCount; i++)
	{
		if (_hmdViews[i] != nullptr)
		{
			// Stop the hmd stream
			PSMRequestID request_id;
			PSM_StopHmdDataStreamAsync(_hmdViews[i]->HmdID, &request_id);
			PSM_EatResponse(request_id);

			// Free out controller listener
			PSM_FreeHmdListener(_hmdViews[i]->HmdID);
			_hmdViews[i] = nullptr;
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

	if (_context)
	{
		// Send message back to driver for logging
		_context->LogMessage(msg);
	}
}
