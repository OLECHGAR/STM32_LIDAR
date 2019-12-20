/**
*@file: Lidar_Motor.c
*@brief: Foctions pour le lidar et le moteur
*@author: LECHGAR Othman, © 2019
**/
/*Includes-------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "Motor.h"
#include "lcd_i2cModule.h"
/*---------------------------------------------------------------*/	

/* External variables -------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/*---------------------------------------------------------------*/	


/* Fontions -----------------------------------------------------*/
/****
Ci : Consigne initiale (ce qu'on veut qu'il fasse)
e : erreur entre la consigne initiale et la réalité
C : Consigne appliquée au moteur
R : Grandeur réelle mesurée (réalité)
*****/

double PID(long Ci,long R,long Old_R)
{
  long P,D,e,I;
  long C;
  e = Ci-R;
  P=e;//Terme Proportionnel
  I = I+e;//Terme Integral
  D = R-Old_R;//Terme Dérivé
  C = Kp*P+Ki*I+Kd*D;
	
  return C;
}
long counter()
{
volatile uint32_t encoder_cnt = 0;
volatile uint32_t tour =0;
volatile float nb_tour_par_sec = 12;
float frequence_codeuse ;
float tick_par_tour_codeuse =12;
float rapport_reducteur =29 ;
float frequence_echantillonnage = 50;
volatile uint32_t tick =0;
	
		encoder_cnt =__HAL_TIM_GET_COUNTER(&htim3);
		
		tick = encoder_cnt;
		//encoder_cnt = 0;
		
		 frequence_codeuse = frequence_echantillonnage*tick;
		
  nb_tour_par_sec = frequence_codeuse/tick_par_tour_codeuse/rapport_reducteur;
	
return nb_tour_par_sec;
}


float Motor(float distance)
{
	
	int Ci,C,R,Old_R = 0;
	if(distance>=100)
		{
			Ci=100;	
		}
		if(distance<100 && distance >=50)
		{
			Ci=80;
		}
		if(distance<50 && distance>=25)
		{
			Ci=60;
		}
		if(distance<10)
		{
			Ci=0;
		}
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Ci);
		
		// fonction pour recuperer R A faire par tarin: Grandeur réelle mesurée (réalité) a partir de l'encodeur 
//  		R=counter();
//		
//		 C = PID(Ci,R,Old_R); // recuperation de la nouvelle commande aprés la gestion d'erreur
//		 Old_R = R;//On sauvegarde

//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, C);//correction de la commande    
return Ci ;
}
void afficher(float distance , float vitesse)
{
  LCD_BackLight(LCD_BL_ON);
	LCD_SetCursor(1,1);
  LCD_Print("vitesse:%.2f m/s",vitesse); //Example of printing float value 
  LCD_SetCursor(2,1);	
	LCD_Print("distance:%.2f cm",distance); //Example of  printing integer value
  HAL_Delay(1000);
  LCD_Clear();
	HAL_Delay(20);
}



 
/*---------------------------------------------------------------*/	
