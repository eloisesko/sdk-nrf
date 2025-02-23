/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "pwm_device.h"

#include <lib/support/CodeUtils.h>

#include <drivers/pwm.h>
#include <logging/log.h>
#include <zephyr.h>

LOG_MODULE_DECLARE(app, CONFIG_MATTER_LOG_LEVEL);

int PWMDevice::Init(const device *aPWMDevice, uint32_t aPWMChannel, uint8_t aMinLevel, uint8_t aMaxLevel,
		    uint8_t aDefaultLevel)
{
	mState = kState_On;
	mMinLevel = aMinLevel;
	mMaxLevel = aMaxLevel;
	mLevel = aDefaultLevel;
	mPwmDevice = aPWMDevice;
	mPwmChannel = aPWMChannel;

	if (!device_is_ready(mPwmDevice)) {
		LOG_ERR("PWM device %s is not ready", mPwmDevice->name);
		return -ENODEV;
	}

	Set(false);
	return 0;
}

void PWMDevice::SetCallbacks(PWMCallback aActionInitiatedClb, PWMCallback aActionCompletedClb)
{
	mActionInitiatedClb = aActionInitiatedClb;
	mActionCompletedClb = aActionCompletedClb;
}

bool PWMDevice::InitiateAction(Action_t aAction, int32_t aActor, uint8_t *aValue)
{
	/* TODO: this function is called InitiateAction because we want to implement some features such as ramping up
	 * here. */
	bool action_initiated = false;
	State_t new_state;

	/* Initiate On/Off Action only when the previous one is complete. */
	if (mState == kState_Off && aAction == ON_ACTION) {
		action_initiated = true;
		new_state = kState_On;
	} else if (mState == kState_On && aAction == OFF_ACTION) {
		action_initiated = true;
		new_state = kState_Off;
	} else if (aAction == LEVEL_ACTION && *aValue != mLevel) {
		action_initiated = true;
		if (*aValue == 0) {
			new_state = kState_Off;
		} else {
			new_state = kState_On;
		}
	}

	if (action_initiated) {
		if (mActionInitiatedClb) {
			mActionInitiatedClb(aAction, aActor);
		}

		if (aAction == ON_ACTION || aAction == OFF_ACTION) {
			Set(new_state == kState_On);
		} else if (aAction == LEVEL_ACTION) {
			mState = new_state;
			SetLevel(*aValue);
		}

		if (mActionCompletedClb) {
			mActionCompletedClb(aAction, aActor);
		}
	}

	return action_initiated;
}

void PWMDevice::SetLevel(uint8_t aLevel)
{
	LOG_INF("Setting brightness level to %u", aLevel);
	mLevel = aLevel;
	UpdateLight();
}

void PWMDevice::Set(bool aOn)
{
	mState = aOn ? kState_On : kState_Off;
	UpdateLight();
}

void PWMDevice::UpdateLight()
{
	constexpr uint32_t kPwmWidthUs = 20000u;
	const uint8_t maxEffectiveLevel = mMaxLevel - mMinLevel;
	const uint8_t effectiveLevel =
		mState == kState_On ? chip::min<uint8_t>(mLevel - mMinLevel, maxEffectiveLevel) : 0;

	pwm_pin_set_usec(mPwmDevice, mPwmChannel, kPwmWidthUs, kPwmWidthUs * effectiveLevel / maxEffectiveLevel, 0);
}
