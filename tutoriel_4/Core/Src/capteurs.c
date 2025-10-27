/*
 * capteurs.c
 *
 *  Created on: Oct 18, 2025
 *      Author: fares
 */


#include "capteurs.h"



Sensor_values sensors ;

void init_sensors(){

	for(int i = 0; i < 3; i++){
    	sensors.gyro.gyro_valeure_entiere[i] = 0;
    	sensors.gyro.gyro_valeure_decimal[i] = 0;
    	sensors.gyro.gyro_offset[i] = 0;
    	sensors.accel.accelXYZ[i] = 0;
    }

    sensors.coordinates.latitude = 0.0f;
    sensors.coordinates.longitude = 0.0f;

    sensors.freq_cardiaque = 0;

    calibrate_sensors() ;

}


void calibrate_sensors() {
    int32_t gyro_sum[3] = {0};

    printf("=== Début de la calibration... ===\r\n");

    for (int j = 0; j < 3; j++) {
        set_gyro_offset(j, 0);
    }

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float gyro[3];
        BSP_GYRO_GetXYZ(gyro);

        for (int j = 0; j < 3; j++) {
            gyro_sum[j] += (int32_t)gyro[j];
        }

        HAL_Delay(10);
    }

    for (int j = 0; j < 3; j++) {
        set_gyro_offset(j, (int)(gyro_sum[j] / CALIBRATION_SAMPLES));
    }

    printf("=== Calibration terminée ===\r\n");
    printf("Gyro offset : X=%d, Y=%d, Z=%d\r\n",
            get_gyro_offset(0), get_gyro_offset(1), get_gyro_offset(2));
}





int get_gyro_int(int axis){          // axe: 0=x,1=y,2=z
    if(axis >= 0 && axis < 3)
        return sensors.gyro.gyro_valeure_entiere[axis];
    return 0;
}

float get_gyro_float(int axis){
    if(axis >= 0 && axis < 3)
        return sensors.gyro.gyro_valeure_decimal[axis];
    return 0.0f;
}

int get_accel(int axis){
    if(axis >= 0 && axis < 3)
        return sensors.accel.accelXYZ[axis];
    return 0;
}

int get_gyro_offset(int axis){
    if(axis >= 0 && axis < 3)
        return sensors.gyro.gyro_offset[axis];
    return 0;
}


float get_latitude(){
    return sensors.coordinates.latitude;
}

float get_longitude(){
    return sensors.coordinates.longitude;
}

int get_freq_cardiaque(){
    return sensors.freq_cardiaque;
}

void set_gyro_int(int axis, int value){
    if(axis >= 0 && axis < 3)
        sensors.gyro.gyro_valeure_entiere[axis] = value;
}

void set_gyro_float(int axis, int value){
    if(axis >= 0 && axis < 3)
        sensors.gyro.gyro_valeure_decimal[axis] = value;
}

void set_gyro_offset(int axis, int value){
    if(axis >= 0 && axis < 3)
        sensors.gyro.gyro_offset[axis] = value;
}

void set_accel(int axis, int value){
    if(axis >= 0 && axis < 3)
        sensors.accel.accelXYZ[axis] = value;
}

void set_latitude(float value){
    sensors.coordinates.latitude = value;
}

void set_longitude(float value){
    sensors.coordinates.longitude = value;
}

void set_freq_cardiaque(int value){
    sensors.freq_cardiaque = value;
}



void setGyroXYZ(){
	float pGyroDataXYZ[3] = {0};
	BSP_GYRO_GetXYZ(pGyroDataXYZ);
	float gyroFrac ;

	for(int i = 0 ; i < 3 ; i++){

		set_gyro_int(i, pGyroDataXYZ[i] - get_gyro_offset(i)) ;
		gyroFrac =  pGyroDataXYZ[i] - get_gyro_int(i);
		set_gyro_float(i, (int) trunc(gyroFrac * 100)) ;
	}
}
void setAccelXYZ(){
	int16_t pDataXYZ[3] = {0};
	BSP_ACCELERO_AccGetXYZ(pDataXYZ);
	for(int i = 0 ; i < 3 ; i++){
		set_accel(i, pDataXYZ[i]) ;
	}
}
void setFreq(){
	// A faire
	// obtenir la valeur du capteur venant d'un ADC

	//set_freq_cardiaque(valeur_frequence_cardiaque)

}
void setGPSCoordinate(){
	// A faire
	// obtenir la valeur du capteur venant d'un ADC

	//set_longitude(valeur_longitude)
	//set_latitude(valeur_latitude)

}




