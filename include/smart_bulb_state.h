#pragma once

enum class State : uint8_t
{
    DAY_OFF = 0,
    DAY_ON,
    NIGHT_OFF,
    NIGHT_ON,
};

enum class Event
{
    SENSOR_LOW,
    SENSOR_HIGH,
    BUTTON,
    SHAKE,
    TIMER
};

//! simple state-machine, giving the new state after an event
static State next_state(State current_state, Event event);

//! apply actions for state-changes 
static void apply_state(State current_state);