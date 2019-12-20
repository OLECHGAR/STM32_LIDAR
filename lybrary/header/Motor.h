/**
*@file: Lidar_Motor.c
*@brief: Foctions pour le lidar et le moteur
*@author: LECHGAR Othman, © 2019
**/
/*Includes-------------------------------------------------------*/
#include "stm32f7xx_hal.h"
/*---------------------------------------------------------------*/	
	
/*Defines and variables------------------------------------------*/

//PID
#define Kp 300 // Coefficient proportionnel
#define Ki 5.5 // Coefficient int�grateur
#define Kd 100 // Coefficient d�rivateur



/*---------------------------------------------------------------*/	

/* signature Fontions -------------------------------------------*/
double PID(long Ci,long R,long Old_R);
long counter();
float Motor(float distance);
void afficher(float distance , float vitesse);
/*---------------------------------------------------------------*/	

