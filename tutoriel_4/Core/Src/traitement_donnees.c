/*
 * traitement_donnees.c
 *
 *  Created on: Oct 19, 2025
 *      Author: fares
 */

#include "traitement_donnees.h"


static FallState fall_state = STATE_IDLE;
static uint32_t impact_time = 0;

float accel_raw_to_g(int raw){
    return (raw / 1000.0f); // -> g
}


float vector_magnitude_g(int ax_raw, int ay_raw, int az_raw) {
    float ax = accel_raw_to_g(ax_raw);
    float ay = accel_raw_to_g(ay_raw);
    float az = accel_raw_to_g(az_raw);
    return sqrtf(ax*ax + ay*ay + az*az);
}


float tilt_ratio(int ax, int ay, int az) {
    float norm = vector_magnitude_g(ax, ay, az);
    if (norm < 1e-6f) return 0.0f;
    return fabsf(az) / norm;
}


bool detect_fall(uint32_t current_time_ms) {
    int ax = get_accel(0);
    int ay = get_accel(1);
    int az = get_accel(2);

    float accel_g = vector_magnitude_g(ax,ay,az);

    switch (fall_state) {
		case STATE_IDLE:
			if (accel_g < FREE_FALL_THRESHOLD) fall_state = STATE_FREEFALL;
			break;
		case STATE_FREEFALL:
			if (accel_g > IMPACT_THRESHOLD) {
				fall_state = STATE_IMPACT; impact_time = current_time_ms;
			}
			else if (accel_g > 0.9f) {
				/* cas: fausse alerte */
				fall_state = STATE_IDLE;
			}
			break;
		case STATE_IMPACT:
			if (current_time_ms - impact_time > STABLE_TIME_MS) {
				if (tilt_ratio(ax,ay,az) < STABLE_TILT_THRESHOLD) {
					fall_state = STATE_STABLE;
					return true;
				}
				else fall_state = STATE_IDLE;
			}
			break;
		case STATE_STABLE:
			if (accel_g > 0.9f && tilt_ratio(ax,ay,az) > 0.9f) {
				fall_state = STATE_IDLE;
			}
			break;
		}
    return false;
}


