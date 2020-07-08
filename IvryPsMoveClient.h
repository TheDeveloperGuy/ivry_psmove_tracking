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

#ifndef _IVRY_PSMOVE_CLIENT_H
#define _IVRY_PSMOVE_CLIENT_H
#include "PSMoveClient_CAPI.h"
#include <windows.h>
#include <openvr_driver.h>
#include <chrono>

#define FPS_REPORT_DURATION 500 // ms

#define PSMOVE_DEFAULT_TRACKING_RATE 120
#define PSMOVE_MAX_CONTROLLERS 4
#define PSMOVE_SCALE_FACTOR (0.01f)

enum eDeviceType
{
    _deviceTypeHMD = 1,
    _deviceTypeController = 2,
};

class IvryPsMoveClient
{
public:
	/** Constructor/destructor **/
	IvryPsMoveClient(
		eDeviceType deviceType,
		int32_t deviceCount,
		int32_t deviceIDs[],
		PSMTrackingColorType bulbColors[],
		bool sendSensorData = true,
		int triggerAxisIndex = -1);
	~IvryPsMoveClient();

	/** Open/close tracking **/
	bool open();
	void close();

	/** Tracking main loop**/
	void run();

	/** Start/stop tracking thread **/
	bool enable_tracking(bool enable = true);

	/** Set tracking rate **/
	void set_tracking_rate(float rate) { _trackingRate = rate; }

	/** Set controller serial number filter **/
	void set_controller_serialno_filter(const char *serialNo) { _controllerSerialNoFilter = serialNo; }

	/** Set callbacks **/
	void set_tracking_callback(void(*callback)(vr::DriverPose_t&, void*), void *context = NULL) { _trackingCallback = callback; _trackingCallbackContext = context; }
	void set_controller_callback(void(*callback)(vr::DriverPose_t&, uint16_t, float, void*), void *context = NULL) { _controllerCallback = callback; _controllerCallbackContext = context; }
	void set_log_callback(void(*callback)(const char*, void*), void *context = NULL) { _logCallback = callback; _logCallbackContext = context; }

	/** Log message **/
	void log_msg(const char *format, ...);

	/** Get last error code **/
	DWORD get_last_error() { return _lastError; }

private:
	/** PSMoveService client event handlers **/
	void handle_client_psmove_event(PSMEventMessage::eEventType event_type);

	void handle_acquire_hmd(PSMResult resultCode, PSMHmdID trackedHmdIndex);
	void handle_acquire_controller(PSMResult resultCode, PSMControllerID trackedControllerIndex);

	/** Start PSMoveService client **/
	bool startup();

	/** Update PSMoveService tracking **/
	void update();

	/** Stop PSMoveService client **/
	void shutdown();

	/** Check if PSMoveService is running **/
	bool is_psmoveservice_running();

	/** Tracking thread entry point **/
	static DWORD WINAPI sensor_thread_proc(void *arg);

	/** Client has been started up? **/
	bool _isOpen;

	/** Thread and timer handles **/
	HANDLE _sensorThread;
	HANDLE _sensorTimer;
	BOOL _sensorActive;

	/** Last error code **/
	DWORD _lastError;

	/** Callbacks **/
	void(*_trackingCallback)(vr::DriverPose_t& pose, void* context);
	void *_trackingCallbackContext;
	void(*_controllerCallback)(vr::DriverPose_t& pose, uint16_t buttons, float trigger, void* context);
	void *_controllerCallbackContext;
	void(*_logCallback)(const char* error, void* context);
	void *_logCallbackContext;

	/** Tracking rate **/
	float _trackingRate;

	/** Controller serial number to use for HMD tracking **/
	std::string _controllerSerialNoFilter;

	/** PSMoveFreepieBridge members **/
	eDeviceType m_device_type;
	PSMHeadMountedDisplay *hmd_views[PSMOVE_MAX_CONTROLLERS];
	PSMController *controller_views[PSMOVE_MAX_CONTROLLERS];
	std::chrono::milliseconds last_report_fps_timestamp;
	PSMRequestID start_stream_request_ids[PSMOVE_MAX_CONTROLLERS];
	PSMControllerID trackedControllerIDs[PSMOVE_MAX_CONTROLLERS];
	PSMHmdID trackedHmdIDs[PSMOVE_MAX_CONTROLLERS];
	PSMTrackingColorType trackedBulbColors[PSMOVE_MAX_CONTROLLERS];
	int32_t trackedControllerCount;
    int32_t trackedHmdCount;
	bool m_sendSensorData;
    int32_t m_triggerAxisIndex;

	void init_controller_views();
	void free_controller_views();
	void init_hmd_views();
	void free_hmd_views();
};

#endif // _IVRY_PSMOVE_CLIENT_H