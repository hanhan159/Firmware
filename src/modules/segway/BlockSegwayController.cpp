#include "BlockSegwayController.hpp"

void BlockSegwayController::update() {
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) return; // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) return;

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) updateParams();

	// get new information from subscriptions
	updateSubscriptions();

	// default all output to zero unless handled by mode
	for (unsigned i = 2; i < NUM_ACTUATOR_CONTROLS; i++)
		_actuators.control[i] = 0.0f;

	// only update guidance in auto mode
	if (_status.main_state == MAIN_STATE_AUTO) {
		// update guidance
	}

	// compute duty cycle command
	float dutyCycle = -th2v.update(_att.pitch) - 
		q2v.update(_att.pitchspeed);

	// motor commands to compute based on state
	float motorLeft = 0;
	float motorRight = 0;

	// handle autopilot modes
	if (_status.main_state == MAIN_STATE_AUTO ||
	    _status.main_state == MAIN_STATE_SEATBELT ||
	    _status.main_state == MAIN_STATE_EASY) {
		// TODO should mix guidance here
		motorLeft = dutyCycle;
		motorRight = dutyCycle;

	} else if (_status.main_state == MAIN_STATE_MANUAL) {
		// TODO should mix manual
		motorLeft = dutyCycle;
		motorRight = dutyCycle;
	}
	
	// send commands if armed
	if (_status.arming_state == ARMING_STATE_ARMED) {
		_actuators.control[0] = motorLeft;
		_actuators.control[1] = motorRight;
	} else {
		_actuators.control[0] = 0;
		_actuators.control[1] = 0;
	}

	// update all publications
	updatePublications();

}

