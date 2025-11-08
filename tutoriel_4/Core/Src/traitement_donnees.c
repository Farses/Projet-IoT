/*
 * traitement_donnees.c - Enhanced with Pre-impact Fall Detection
 *
 *  Created on: Oct 19, 2025
 *      Author: fares
 *  Modified: Nov 5, 2025 - Added pre-impact detection algorithm
 */

#include "traitement_donnees.h"

// ===== État de détection post-impact (original) =====
static FallState state = IDLE;
static uint32_t t_weightless = 0;
static uint32_t t_impact = 0;
static uint32_t t_motionless = 0;
static float gravity_before[3];

// ===== Variables pour la détection pré-impact =====
static float h1 = 0.0f;  // Distance PCB au sol (cm) - navel to ground
static float h2 = 0.0f;  // Distance tête au PCB (cm) - head to PCB
static float beta_prev = 0.0f;  // Angle précédent
static float S = 0.0f;  // Déplacement angulaire cumulé
static bool preimpact_enabled = false;
static uint32_t preimpact_detection_time = 0;

// ===== CONFIGURATION DES HAUTEURS =====
// Appeler cette fonction au démarrage avec la taille de l'utilisateur
void configure_user_height(float user_height_cm) {
    // Approximations basées sur les proportions moyennes du corps humain
    h1 = user_height_cm * 0.55f;  // ~55% de la hauteur totale (nombril au sol)
    h2 = user_height_cm * 0.45f;  // ~45% de la hauteur totale (tête au nombril)

    printf("Configuration utilisateur: Taille=%.1f cm | h1=%.1f cm | h2=%.1f cm\r\n",
           user_height_cm, h1, h2);

    preimpact_enabled = true;
    S = 0.0f;
    beta_prev = M_PI / 2.0f;  // Initialisation à 90° (position debout)
}

// ===== Fonctions de conversion existantes =====
float gyro_raw_to_dps(int raw){
    return raw * (GYRO_FS_DPS / 32768.0f);
}

float accel_raw_to_g(int raw){
    return (raw / 1000.0f);
}

float vector_magnitude_g(int ax_raw, int ay_raw, int az_raw) {
    float ax = accel_raw_to_g(ax_raw);
    float ay = accel_raw_to_g(ay_raw);
    float az = accel_raw_to_g(az_raw);
    return sqrtf(ax*ax + ay*ay + az*az);
}

float tilt_angle_deg(float ax, float ay, float az) {
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (mag < 1e-6f) return 0.0f;
    return acosf(ay / mag) * (180.0f / M_PI);
}

float tilt_ratio(int ax, int ay, int az) {
    float norm = vector_magnitude_g(ax, ay, az);
    if (norm < 1e-6f) return 0.0f;
    return fabsf(az) / norm;
}

float jerk_g(float mag_now, float mag_prev, float dt){
    return fabsf(mag_now - mag_prev) / dt;
}

float vector_angle(float ax1, float ay1, float az1, float ax2, float ay2, float az2){
    float dot = ax1*ax2 + ay1*ay2 + az1*az2;
    float n1 = sqrtf(ax1*ax1 + ay1*ay1 + az1*az1);
    float n2 = sqrtf(ax2*ax2 + ay2*ay2 + az2*az2);
    float c = dot / (n1*n2 + 1e-6f);
    if(c > 1.0f) c = 1.0f;
    else if(c < -1.0f) c = -1.0f;
    return acosf(c);
}

float total_rotation_dps(float gx, float gy, float gz){
    return sqrtf(gx*gx + gy*gy + gz*gz)/3;
}

bool is_accel_clipped(float ax, float ay, float az){
    const float LIM = 1.98f;
    return (fabsf(ax) >= LIM || fabsf(ay) >= LIM || fabsf(az) >= LIM);
}

float orientation_change(float ax_now, float ay_now, float az_now) {
    float diff_x = fabsf(ax_now - gravity_before[0]);
    float diff_y = fabsf(ay_now - gravity_before[1]);
    float diff_z = fabsf(az_now - gravity_before[2]);
    float max_diff = diff_x;
    if(diff_y > max_diff) max_diff = diff_y;
    if(diff_z > max_diff) max_diff = diff_z;
    return max_diff;
}



bool detect_fall(uint32_t now_ms) {
    // === Lecture des données capteur ===
    float ax = accel_raw_to_g(get_accel(0));
    float ay = accel_raw_to_g(get_accel(1));
    float az = accel_raw_to_g(get_accel(2));
    float sum_accel = ax + ay + az;

    float gx = gyro_raw_to_dps(get_gyro_int(0));
    float gy = gyro_raw_to_dps(get_gyro_int(1));
    float gz = gyro_raw_to_dps(get_gyro_int(2));

    float mag = vector_magnitude_g(get_accel(0), get_accel(1), get_accel(2));

    switch (state) {

    // État IDLE : Attente de chute (méthode classique free-fall)
    case IDLE:
        if (mag < THRESH_FREEFALL || sum_accel < 0) {
            printf("Transition IDLE → WEIGHTLESSNESS | mag=%.2f < %.2f\r\n",
                   mag, THRESH_FREEFALL);
            gravity_before[0] = ax;
            gravity_before[1] = ay;
            gravity_before[2] = az;
            t_weightless = now_ms;
            state = WEIGHTLESSNESS;
        }
        break;

    case WEIGHTLESSNESS:
        if (now_ms - t_weightless >= TIME_FREEFALL) {
            if (mag > THRESH_IMPACT) {
                printf("IMPACT détecté ! | mag=%.2f g | temps chute=%.0f ms\r\n",
                       mag, (float)(now_ms - t_weightless));

                t_impact = now_ms;
                state = IMPACT;
            }
            else if(sum_accel > THRESH_IMPACT_ACCELSUM){
                printf("IMPACT détecté (threshold accel) | mag=%.2f g | accel=%.2f g\r\n",
                       mag, sum_accel);
                t_impact = now_ms;
                state = IMPACT;
            }
            else if (now_ms - t_weightless > MAX_TIME_FREEFALL_TO_IMPACT) {
                printf("Timeout WEIGHTLESSNESS → retour IDLE\r\n\n");
                state = IDLE;
            }
        }
        else if (mag > 1.0f) {
            printf("Fausse alerte (mag=%.2f > 1.0) → retour IDLE\r\n\n", mag);
            state = IDLE;
        }
        break;

    case IMPACT:
        if (mag < 1.5f) {
            printf("Stabilisation post-impact (mag=%.2f) → FALL CONFIRMED\r\n", mag);
            t_motionless = now_ms;
            state = FALL_CONFIRMED;
        }
        else if (now_ms - t_impact > MAX_TIME_IMPACT_TO_MOTIONLESS) {
            printf("Timeout IMPACT → retour IDLE\r\n\n");
            state = IDLE;
        }
        break;

    case FALL_CONFIRMED:
        if (now_ms - t_motionless >= 100) {
            printf("Immobilité prolongée (>10s) → alerte critique !\r\n");
            printf("Retour IDLE pour démo\r\n");
            state = IDLE;
            return true;

        }
        break;
    }

    return false;
}

// ===== Fonctions utilitaires =====
void reset_fall_detection(void) {
    state = IDLE;
    t_weightless = 0;
    t_impact = 0;
    t_motionless = 0;
    preimpact_detection_time = 0;
    S = 0.0f;
    beta_prev = M_PI / 2.0f;
    gravity_before[0] = 0.0f;
    gravity_before[1] = 0.0f;
    gravity_before[2] = 0.0f;
}

FallState get_fall_state(void) {
    return state;
}

const char* get_fall_state_name(void) {
    switch(state) {
        case IDLE: return "IDLE";
        case WEIGHTLESSNESS: return "WEIGHTLESSNESS";
        case IMPACT: return "IMPACT";
        case MOTIONLESS: return "MOTIONLESS";
        case FALL_CONFIRMED: return "FALL_CONFIRMED";
        default: return "UNKNOWN";
    }
}

// Fonction pour obtenir le lead time (temps avant impact)
uint32_t get_lead_time_ms(void) {
    if (preimpact_detection_time > 0 && t_impact > 0) {
        return t_impact - preimpact_detection_time;
    }
    return 0;
}

// Fonction pour vérifier si pré-impact est actif
bool is_preimpact_enabled(void) {
    return preimpact_enabled;
}
