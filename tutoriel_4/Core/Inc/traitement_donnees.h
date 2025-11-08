/*
 * traitement_donnees.h
 *
 *  Created on: Oct 19, 2025
 *      Author: fares
 */

#ifndef INC_TRAITEMENT_DONNEES_H_
#define INC_TRAITEMENT_DONNEES_H_

#include "capteurs.h"

#define THRESH_FREEFALL    0.2f     // 0.75g - seuil de chute libre
#define TIME_FREEFALL      50 // 30ms - durée minimale chute libre
#define THRESH_IMPACT      1.5f      // 2.0g - seuil d'impact
#define THRESH_IMPACT_ACCELSUM      1.8f      // 2.0g - seuil d'impact

#define THRESH_MOTIONLESS  0.1875f   // 0.1875g - seuil d'immobilité
#define TIME_MOTIONLESS    1000      // 2000ms - durée immobilité
#define THRESH_ORIENTATION 0.7f      // 0.7g - changement d'orientation


#define MAX_TIME_FREEFALL_TO_IMPACT  2000   // Max 2s entre chute et impact
#define MAX_TIME_IMPACT_TO_MOTIONLESS 1000  // Max 1s entre impact et immobilité


#define ACC_FS_G      2.0f  // ±2g
#define GYRO_FS_DPS   245.0f // selon config BMI
#define G_TO_MS2      9.80665f


typedef enum {
    IDLE,              // État initial/normal
    WEIGHTLESSNESS,    // Détection de chute libre (FREE_FALL)
    IMPACT,            // Détection d'impact (ACTIVITY)
    MOTIONLESS,        // Détection d'immobilité (INACTIVITY)
    FALL_CONFIRMED     // Chute confirmée
} FallState;


// ===== TYPES D'ACTIVITÉS =====
typedef enum {
    ACTIVITY_UNKNOWN = 0,
    ACTIVITY_STANDING,   // Debout immobile
    ACTIVITY_WALKING,    // Marche
    ACTIVITY_SITTING,    // Assis
    ACTIVITY_LYING       // Allongé
} ActivityType;


// Détection
ActivityType detect_activity(float mag, float ay, float az, uint32_t now_ms);

// Conversions

float gyro_raw_to_dps(int raw) ;
float accel_raw_to_g(int raw);

float vector_magnitude_g(int ax_raw, int ay_raw, int az_raw);


float tilt_ratio(int ax, int ay, int az);
float tilt_angle_deg(float ax, float ay, float az) ;

float jerk_g(float mag_now, float mag_prev, float dt) ;
float vector_angle(float ax1, float ay1, float az1, float ax2, float ay2, float az2) ;

float total_rotation_dps(float gx, float gy, float gz) ;

bool detect_preimpact_fall(float ax, float ay, float az, float mag)  ;
void configure_user_height(float user_height_cm)  ;

bool is_accel_clipped(float ax, float ay, float az);



// Gestion état
void reset_fall_detection(void);
FallState get_fall_state(void);
const char* get_fall_state_name(void);
ActivityType get_current_activity() ;
const char* get_activity_name(ActivityType activity);

// Buffer analysis
void update_buffer(float mag, float ay);
float calculate_variance(float* buffer, int size);
float calculate_mean(float* buffer, int size);



#endif /* INC_TRAITEMENT_DONNEES_H_ */
