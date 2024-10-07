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

#ifndef _IVRY_PSMOVE_TRACKING_APP_H
#define _IVRY_PSMOVE_TRACKING_APP_H

#include "IvryTrackingApp.h"
#include "IvryPsMoveClient.h"

class IvryPsMoveTrackingApp : public IvryTrackingApp
{
public:
	/** Constructor/destructor **/
	IvryPsMoveTrackingApp();
	~IvryPsMoveTrackingApp();

	/** Run tracker **/
	virtual DWORD Run();

protected:
	/** Send pose to driver **/
	virtual void PoseUpdated(const vr::DriverPose_t &pose);

	/** Controller haptics request has been received from driver **/
	virtual void OnControllerHaptics(uint32_t id, uint32_t component, float fDurationSeconds, float fFrequency, float fAmplitude);

	/** Pose has been recevied from driver **/
	virtual void OnDevicePoseUpdated(const vr::DriverPose_t &pose);

	/** Get min/max tracking rates (in Hz) **/
	virtual float GetMinTrackingRate();
	virtual float GetMaxTrackingRate();

	/** Get/Set tracking rate (in Hz) **/
	virtual float GetTrackingRate();
	virtual void SetTrackingRate(float rate);

	/** Device orientation has been enabled/disabled by user **/
	virtual void OnDeviceOrientationEnabled(bool enable);

	/** Driver has recentered headset **/
	virtual void OnDeviceRecenter();

	/** Reload tracker settings **/
	virtual void OnReloadSettings();

	/** Driver is requesting tracking process quit **/
	virtual void OnQuit();

	/** Check if PSMoveService is running **/
	bool IsPsMoveServiceRunning();

private:
	/** PSMove client tracking instance **/
	friend class IvryPsMoveClient;
	IvryPsMoveClient *m_pPsMoveClient;

	/** Handle to PSMoveService process **/
	HANDLE m_hPsMoveService;

	/** Tracking has been enabled already? **/
	bool m_bTrackingEnabled;

	/** Use device orientation tracking? **/
	bool m_bUseDeviceOrientation;

	/** Device center offset from PSMoveService center **/
	double m_fDeviceCenterOffset;

	/** Device has been recentered **/
	bool m_bDeviceWasRecentered;
};

#endif // _IVRY_PSMOVE_TRACKING_APP_H