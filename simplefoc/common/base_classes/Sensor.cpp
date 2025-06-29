#include "Sensor.h"
#include "foc_utils.h"
#include "time_utils.h"



void Sensor::update() {
    float val = getSensorAngle();
    if (val<0) // sensor angles are strictly non-negative. Negative values are used to signal errors.
        return; // TODO signal error, e.g. via a flag and counter
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

void Sensor::Outupdate() {
    float val = getOutSensorAngle();
    // Serial.println(val);
    angle_prev_tsOut = _micros();
    float d_angle = val - angle_prevOut;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotationsOut += ( d_angle > 0 ) ? -1 : 1; 
    // Serial.println(full_rotations);
    angle_prevOut = val;
}

 /** get current angular velocity (rad/s) */
float Sensor::getVelocity() {
    // calculate sample time
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    if (Ts < 0.0f) {    // handle micros() overflow - we need to reset vel_angle_prev_ts
        vel_angle_prev = angle_prev;
        vel_full_rotations = full_rotations;
        vel_angle_prev_ts = angle_prev_ts;
        return velocity;
    }
    if (Ts < min_elapsed_time) return velocity; // don't update velocity if deltaT is too small

    velocity = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return velocity;
}

float Sensor::getVelocityOut() {
    // calculate sample time
    float Ts = (angle_prev_tsOut - vel_angle_prev_tsOut)*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0) Ts = 1e-3f;
    // velocity calculation
    float vel = ( (float)(full_rotationsOut - vel_full_rotationsOut)*_2PI + (angle_prevOut - vel_angle_prevOut) ) / Ts;    
    // save variables for future pass
    vel_angle_prevOut = angle_prevOut;
    vel_full_rotationsOut = full_rotationsOut;
    vel_angle_prev_tsOut = angle_prev_tsOut;
    return vel;
}

void Sensor::init() {
    // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
    getSensorAngle(); // call once
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); // call again
    vel_angle_prev_ts = _micros();
    delay(1);
    getSensorAngle(); // call once
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); // call again
    angle_prev_ts = _micros();
}


float Sensor::getMechanicalAngle() {
    return angle_prev;
}



float Sensor::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor::getOutAngle(){
    return (float)full_rotationsOut * _2PI + angle_prevOut;
}

double Sensor::getPreciseAngle() {
    return (double)full_rotations * (double)_2PI + (double)angle_prev;
}



int32_t Sensor::getFullRotations() {
    return full_rotations;
}



int Sensor::needsSearch() {
    return 0; // default false
}
