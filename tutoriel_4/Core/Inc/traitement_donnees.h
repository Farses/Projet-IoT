/*
 * traitement_donnees.h
 *
 *  Created on: Oct 19, 2025
 *      Author: fares
 */

#ifndef INC_TRAITEMENT_DONNEES_H_
#define INC_TRAITEMENT_DONNEES_H_

#include "capteurs.h"


#define FREE_FALL_THRESHOLD     0.4f   // < 0.4g ≈ chute libre
#define IMPACT_THRESHOLD        2.0f   // > 2g ≈ impact
#define STABLE_TILT_THRESHOLD   0.7f   // cos(angle) threshold
#define STABLE_TIME_MS          1000   // ms
#define ACCEL_SENSITIVITY_MG_PER_LSB   0.244


typedef enum { STATE_IDLE, STATE_FREEFALL, STATE_IMPACT, STATE_STABLE } FallState;



float accel_raw_to_g(int raw);

float vector_magnitude_g(int ax_raw, int ay_raw, int az_raw);


float tilt_ratio(int ax, int ay, int az);


#endif /* INC_TRAITEMENT_DONNEES_H_ */
