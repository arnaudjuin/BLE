/**
 ******************************************************************************
 * @file    Source/App/Motion/motion.c
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Motion component implementation file
 ******************************************************************************
 * @attention
 *
 * &copy; COPYRIGHT(c) 2017 Arthur Burnichon - arthur@meetluseed.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motion.h"

#include "debug.h"
#include <math.h>   /* trunc */
#include "hal_conf.h"
/* Private define ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Motion initialization function
  * @param  None
  * @retval None
  */

struct angle
   {
   int32_t x;
   int32_t y;
   int32_t z;
   int32_t previous_x;
   int32_t previous_y;
   int32_t previous_z;
   int maj;
   int start;
   };

struct angle a;


void Motion_Init(void) {
a.start=0;
Mems_Humidity_Enable();
Mems_Temperature_Enable();
Mems_Pressure_Enable();
}

/**
  * @brief  Processing function to detect motion event
  * @param  None
  * @retval None
  */
void Motion_Process(void) {

// On met à jour les valeurs en prenant en compte l'hystérysis

	if(a.start==0)
	{
		a.previous_x=a.x+51;				///On sauvegarde les valeurs de départ au début du programme
		a.previous_y=a.y+51;
		a.previous_z=a.z+51;
		a.start=1;
	}

	if (((abs(a.x-a.previous_x))>50)||(abs(a.y-a.previous_y)>50)||(abs(a.z-a.previous_z))>50)			///Condition d'hysterisis (anti bounce effect), on ne rentre dans le if que si une des 3 valeurs d'accélerations grandit sensiblement
	{

			Accelero_Status_Change_Callback();						///On profite de l'hysterysy que l'on a codé auparavant pour mettre a jour les valeurs de accelero BLE
		///On calcule ici les valeurs des angles grâce à l'accèléromètre
	  	float total = sqrt(a.x*a.x+a.y*a.y+a.z*a.z);			///Vecteur unitaire


	  		 int angleX = (int)(asin(a.x/ total )*180.0/3.1416);			///Conversion des accélérations en angle
	  		 int angleY = (int)(asin(a.y/ total )*180.0/3.1416);
	  		 int angleZ = (int)(acos(a.z/ total )*180.0/3.1416);
///MANQUE DE TEMPS POUR FINIR, C'est ICI Qu'il faut apperler la fonction read du service inclinaison
	  	/*
	  		if (angleX>=28 &&  angleX<=32)
	  			Motion_InclinationLeftRightAxisLeftMedium_CallBack();
	  		if (angleX>=-32 &&  angleX<=-28)
	  			Motion_InclinationLeftRightAxisRightMedium_CallBack();					///Ajout d'intervalles
	  		if (angleX>=58 &&  angleX<=62)
	  			Motion_InclinationLeftRightAxisLeftHigh_CallBack();
	  		if (angleX>=-62 &&  angleX<=-58)
	  			Motion_InclinationLeftRightAxisRightHigh_CallBack();
	  		if (angleY>=-2 &&  angleY<=2)
	  			Motion_InclinationBackFrontAxisNone_CallBack();
	  		if (angleY>=28 &&  angleY<=32)
	  			Motion_InclinationBackFrontAxisFrontMedium_CallBack() ;
	  		if (angleY>=58 &&  angleY<=62)
	  		Motion_InclinationBackFrontAxisFrontHigh_CallBack() ;
	  		if (angleY>=-32 &&  angleY<=-28)
	  			Motion_InclinationBackFrontAxisBackMedium_CallBack() ;
	  		if (angleY>=-62 &&  angleY<=-58)
	  			Motion_InclinationBackFrontAxisBackHigh_CallBack();
	  		if (angleX==30 && angleY==30)
	  			Motion_InclinationTopRightMedium_CallBack();
	  		if (angleX==30 && angleY==-30)
	  			Motion_InclinationBackRightMedium_CallBack();
	  		if (angleX==-30 && angleY==30)
	  			Motion_InclinationTopLeftMedium_CallBack() ;
	  		if (angleX==-30 && angleY==-30)
	  			Motion_InclinationBackLeftMedium_CallBack();

	  		a.previous_x=a.x;
	  		a.previous_y=a.y;
	  		a.previous_z=a.z;
*/


	}

}

void Mems_Accelero_Callback(int32_t axis_x, int32_t axis_y, int32_t axis_z)			///Reimplementation du callback en non weak
{
 a.x=axis_x;	///On met à jour les valeur d'angle dans la structure globale
 a.y=axis_y;
 a.z=axis_z;
}
int64_t Mems_Get_Accelero (void)
{
	return a.z;
}
