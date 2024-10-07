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
	PSM_DeviceTypeUnkown = 0,
    PSM_DeviceTypeHMD = 1,
    PSM_DeviceTypeController = 2,
};

/* PSMove buttons */
typedef enum
{
	PSMOVE_BUTTON_PS,
	PSMOVE_BUTTON_START,
	PSMOVE_BUTTON_SELECT,
	PSMOVE_BUTTON_MOVE,
	PSMOVE_BUTTON_CROSS,
	PSMOVE_BUTTON_CIRCLE,
	PSMOVE_BUTTON_SQUARE,
	PSMOVE_BUTTON_TRIANGLE,
	PSMOVE_BUTTON_COUNT
} IvryPsMoveTrackingPsMoveButtons;

/*! Button flags.
 * The values of each button when pressed. The values returned
 * by the button-related functions return an integer. You can
 * use the logical-and operator (&) to check if a button is
 * pressed (in case of psmove_get_buttons()) or if a button
 * state has changed (in case of psmove_get_button_events()).
 *
 * Used by psmove_get_buttons() and psmove_get_button_events().
 **/
enum PSMove_Button 
{
	/**
	 * See comment in psmove_get_buttons() for how this is
	 * laid out in the input report.
	 *
	 * Source:
	 * https://github.com/nitsch/moveonpc/wiki/Input-report
	 **/
	Btn_TRIANGLE = 1 << 4, /*!< Green triangle */
	Btn_CIRCLE = 1 << 5, /*!< Red circle */
	Btn_CROSS = 1 << 6, /*!< Blue cross */
	Btn_SQUARE = 1 << 7, /*!< Pink square */

	Btn_SELECT = 1 << 8, /*!< Select button, left side */
	Btn_START = 1 << 11, /*!< Start button, right side */

	Btn_PS = 1 << 16, /*!< PS button, front center */
	Btn_MOVE = 1 << 19, /*!< Move button, big front button */
	Btn_T = 1 << 20, /*!< Trigger, on the back */
};

/* PSMove emulated trackpad actions */
typedef enum
{
	PSMOVE_TRACKPAD_ACTION_NONE,

	PSMOVE_TRACKPAD_ACTION_TOUCH,
	PSMOVE_TRACKPAD_ACTION_PRESS,

	PSMOVE_TRACKPAD_ACTION_LEFT,
	PSMOVE_TRACKPAD_ACTION_UP,
	PSMOVE_TRACKPAD_ACTION_RIGHT,
	PSMOVE_TRACKPAD_ACTION_DOWN,

	PSMOVE_TRACKPAD_ACTION_UPLEFT,
	PSMOVE_TRACKPAD_ACTION_UPRIGHT,
	PSMOVE_TRACKPAD_ACTION_DOWNLEFT,
	PSMOVE_TRACKPAD_ACTION_DOWNRIGHT,

	PSMOVE_TRACKPAD_ACTION_COUNT
} IvryPsMoveTrackingPsMoveTouchpadAction;

typedef enum
{
	PSMControllerIndex_Left = 0,
	PSMControllerIndex_Right = 1,
	PSMControllerIndex_Count = 2
} PSMControllerHandIndex;

/* PSMove controller state */
struct IvryPsMoveTrackingPsMoveState : public IvryTrackingControllerState
{
	// Button state
	PSMButtonState buttonState[PSMOVE_BUTTON_COUNT];

	// Touchpad emulation state
	struct
	{
		bool wasActive;
		std::chrono::milliseconds pressTimestamp;
		vr::HmdQuaternion_t orientationAtPress;
		double positionAtPress[3];
	} touchpadState;

	// Orientation reset state
	struct
	{
		std::chrono::milliseconds pressTimestamp;
		bool reset;
	} resetState;

	// Haptic state
	struct
	{
		float pendingDurationSecs;
		float pendingAmplitude;
		float pendingFrequency;
		std::chrono::milliseconds lastRumbleSentTimestamp;
		bool lastRumbleSentValid;
	} hapticState;

	// Power state
	struct
	{
		std::chrono::milliseconds lastStatusSentTimestamp;
		PSMBatteryState state;
	} powerState;
};

#define PSM_ControllerId_None -1

class IvryPsMoveTrackingApp;

class IvryPsMoveClient
{
public:
	/** Constructor/destructor **/
	IvryPsMoveClient(IvryPsMoveTrackingApp *context);
	~IvryPsMoveClient();

	/** Open/close tracking **/
	bool open();
	void close();

	/** Tracking main loop**/
	void run();

	/** (Re)load tracker settings **/
	void load_settings();

	/** Start/stop tracking thread **/
	bool enable_tracking(bool enable = true);

	/** Set tracking rate **/
	void set_tracking_rate(float rate) { _trackingRate = rate; }

	/** Set controller serial number filter **/
	void set_controller_serialno_filter(const char *serialNo) { _controllerSerialNoFilter = serialNo; }

	/** Controller haptics request has been received from driver **/
	void controller_haptics(uint32_t id, float fDurationSeconds, float fFrequency, float fAmplitude);

	/** Log message **/
	void log_msg(const char *format, ...);

	/** Get last error code **/
	DWORD get_last_error() { return _lastError; }

private:
	/** PSMoveService client event handlers **/
	void handle_client_psmove_response(const PSMMessage *message);
	void handle_client_psmove_event(const PSMMessage *message);

	void handle_connect_to_service();
	static void handle_service_version_response(const PSMResponseMessage *response, void *userdata);

	void handle_hmd_list_response(const PSMHmdList *hmd_list, const PSMResponseHandle response_handle);
	void handle_controller_list_response(const PSMControllerList *controller_list, const PSMResponseHandle response_handle);

	/** Start PSMoveService client **/
	bool startup();

	/** Update PSMoveService tracking **/
	void update();

	/** Update PSMoveService controller tracking **/
	void update_controllers();

	/** Update controller haptics **/
	void controller_haptic_update(PSMControllerID id, IvryPsMoveTrackingPsMoveState *state);

	/** Update controller power state **/
	void controller_power_update(PSMControllerID id, IvryPsMoveTrackingPsMoveState *state);

	/** Stop PSMoveService client **/
	void shutdown();

	/** Update trackpad emulation **/
	void controller_touchpad_update(int index);

	/** Client has been started up? **/
	bool _isOpen;

	/** Timer handles **/
	HANDLE _sensorTimer;
	BOOL _sensorActive;

	/** Last error code **/
	DWORD _lastError;

	/** Tracking rate **/
	float _trackingRate;

	/** Controller serial number to use for HMD tracking **/
	std::string _controllerSerialNoFilter;

	/** PSMoveFreepieBridge members **/
	eDeviceType _deviceType;
	PSMControllerID _deviceId;
	std::chrono::milliseconds _lastReportFpsTimestamp;
	PSMRequestID _startStreamRequestIds[PSMOVE_MAX_CONTROLLERS];

	PSMHeadMountedDisplay *_hmdViews[PSMOVE_MAX_CONTROLLERS];
	PSMHmdID _trackedHmdIDs[PSMOVE_MAX_CONTROLLERS];
	int32_t _trackedHmdCount;

	PSMController *_controllerViews[PSMOVE_MAX_CONTROLLERS];
	PSMControllerID _trackedControllerIDs[PSMOVE_MAX_CONTROLLERS];
	PSMControllerID _leftControllerId, _rightControllerId;
	PSMTrackingColorType _trackedBulbColors[PSMOVE_MAX_CONTROLLERS];
	int32_t _trackedControllerCount;
	IvryPsMoveTrackingPsMoveState _controllerState[PSMControllerIndex_Count];
	IvryPsMoveTrackingPsMoveTouchpadAction _controllerTouchpadMapping[PSMOVE_BUTTON_COUNT];

	bool _psMoveUseNativeDriver;
	bool _psMoveHapticsEnabled;
	bool _psMoveEmulateTouchpad;
	bool _psMoveTouchpadDelay;

	bool _sendSensorData;

	void init_controller_views();
	void free_controller_views();
	void init_hmd_views();
	void free_hmd_views();

private:
	IvryPsMoveTrackingApp *_context;
};

#endif // _IVRY_PSMOVE_CLIENT_H