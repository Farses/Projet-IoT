/*
 * capteurs.h
 *
 *  Created on: Oct 18, 2025
 *      Author: fares
 */

#ifndef INC_CAPTEURS_H_
#define INC_CAPTEURS_H_

#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include <stdbool.h>
#include <math.h>


#define CALIBRATION_SAMPLES 250

typedef struct{
	int gyro_valeure_entiere[3] ;
	int gyro_valeure_decimal[3] ;
	int gyro_offset[3] ;
}GyroXYZ;

typedef struct{
	int accelXYZ[3] ;
}AccelXYZ;

typedef struct{
	float longitude ;
	float latitude ;
}gps;

typedef struct{
	GyroXYZ gyro ;
	AccelXYZ accel ;
	gps coordinates ;
	int freq_cardiaque ;
}Sensor_values;

// y avoir accès en dehors du fichier .c
// le but de ce fichier .c/.h permet d'avoir un accès facile aux données des capteurs. (de la même façon qu'en C++ en programmation orienté objet)
extern Sensor_values sensors;

void init_sensors();
void calibrate_sensors() ;


int get_gyro_int(int axis);
float get_gyro_float(int axis);
int get_accel(int axis);
float get_latitude();
float get_longitude();
int get_freq_cardiaque();
int get_gyro_offset(int axis);

void set_gyro_int(int axis, int value);
void set_gyro_float(int axis, int value);
void set_accel(int axis, int value);
void set_latitude(float value);
void set_longitude(float value);
void set_freq_cardiaque(int value);
void set_gyro_offset(int axis, int value);

void setGyroXYZ();
void setAccelXYZ();
void setFreq();
void setGPSCoordinate();



#endif /* INC_CAPTEURS_H_ */
