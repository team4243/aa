/*
*******************************************************************************
**
**  FILENAME: parabolic_cmd_shaping.c
**
**  DESCRIPTION:
**		This file contains the function used to generate a parabolic
**			shaped trajectory between two angles for 2 axes,
**			Azimuth and Elevation.
**
**  FUNCTIONS:
**		Init_PCS
**		Parabolic_Cmd_Shaping
**		Sign_Double
**
**  GLOBAL VARIABLES:
**
**  REQUIREMENTS:
**
**  REFERENCES:
**
**  NOTES:
**
*******************************************************************************
**
**  $File:     $
**  $Revision: $Release 3
**  $Date:     $09April2004
**  $Author:   $Jerry Hull
**
*******************************************************************************
*/
/* ----- INCLUDES ----- */
#include <time.h>
#include <math.h>
#include "defines.h"
#include "c_int02.h"
#include "Parabolic_Cmd_Shaping.h"

/* ----- DEFINES ----- */
#define PCS_MAX_RATE_AZ		 100.0
#define PCS_MAX_RATE_EL		 100.0
#define PCS_MAX_ACCEL_AZ	2250.0
#define PCS_MAX_ACCEL_EL	2500.0

/* ----- MACROS ----- */

/* ----- TYPEDEFS ----- */

/* ----- GLOBALS ----- */ 
PCS_Data_In_Struct  g_PCSIn;
PCS_Data_Out_Struct g_PCSOut;

/* ----- LOCALS ----- */
/* Define Parabolic_Cmd_Shaping local variables */
double dPCS_LOS_Init_Ang_Past_Az	= 0.0;
double dPCS_LOS_Init_Ang_Past_El	= 0.0;
double dTheta_Cmd_Past_Az			= 0.0;
double dTheta_Cmd_Past_El			= 0.0;
double dTheta_Init_Az				= 0.0;
double dTheta_Init_El				= 0.0;
double dT_Zero_Az					= 0.0;
double dT_Zero_El					= 0.0;
double dTheta_Az					= 0.0;
double dTheta_El					= 0.0;
double dDelta_Theta_Cmd_Az			= 0.0;
double dDelta_Theta_Cmd_El			= 0.0; 
double dTheta_Vmax_Az				= 0.0;
double dTheta_Vmax_El				= 0.0;
double dT_Vmax_Az					= 0.0;
double dT_Vmax_El					= 0.0;
double dTheta_2Piece_Az				= 0.0;
double dTheta_2Piece_El				= 0.0;
double dT_Decel_Az					= 0.0;
double dT_Decel_El					= 0.0;
double dT_End_Az					= 0.0;
double dT_End_El					= 0.0;
double dT_Az						= 0.0;
double dT_El						= 0.0;
int bSlewing_Az						= FALSE;
int bSlewing_El						= FALSE;

/* ----- FORWARD DECLARATIONS ----- */

/* External references */

/*
*******************************************************************************
**
**   FUNCTION: Init_PCS
**
**   DESCRIPTION:
**		This module initializes the function Parabolic_Cmd_Shaping.
**
**   INPUTS:
**
**   OUTPUTS:
**
**   RETURN:
**
**   RESTRICTIONS:
**      None
**
*******************************************************************************
**
**  Revision History
**
**  $Log: $
**
*******************************************************************************
*/

/*===========================================================================*/
/* Calculate parabolic trajectory commands */
void Init_PCS()
{
	/* Initializing by command*/ 
	g_PCSIn.LOSInit.dAz	 		= IDLE_HOME_ANG_AZ;
	g_PCSIn.LOSInit.dEl	 		= IDLE_HOME_ANG_EL; 
	g_PCSIn.LOSCmd.dAz			= 0.0;
	g_PCSIn.LOSCmd.dEl			= 0.0;  
	g_PCSIn.InitStart.bAz		= 0.0;
	g_PCSIn.InitStart.bEl		= 0.0;
	dPCS_LOS_Init_Ang_Past_Az 	= g_PCSIn.LOSInit.dAz;
	dPCS_LOS_Init_Ang_Past_El 	= g_PCSIn.LOSInit.dEl;
	dT_Vmax_Az 					= PCS_MAX_RATE_AZ / PCS_MAX_ACCEL_AZ;
	dT_Vmax_El 					= PCS_MAX_RATE_EL / PCS_MAX_ACCEL_EL;
	dTheta_Vmax_Az 				= 0.5 * PCS_MAX_ACCEL_AZ * dT_Vmax_Az * dT_Vmax_Az;
	dTheta_Vmax_El 				= 0.5 * PCS_MAX_ACCEL_EL * dT_Vmax_El * dT_Vmax_El;
	dTheta_2Piece_Az 			= 2 * dTheta_Vmax_Az;
	dTheta_2Piece_El 			= 2 * dTheta_Vmax_El;
	dTheta_Cmd_Past_Az 			= g_PCSIn.LOSInit.dAz;
	dTheta_Cmd_Past_El 			= g_PCSIn.LOSInit.dEl;
	dTheta_Init_Az 				= dTheta_Cmd_Past_Az;
	dTheta_Init_El 				= dTheta_Cmd_Past_El;
	dT_Zero_Az 					= 0.0;
	dT_Zero_El 					= 0.0;
	dTheta_Az 					= 0.0;
	dTheta_El 					= 0.0;
	dDelta_Theta_Cmd_Az 		= 0.0;
	dDelta_Theta_Cmd_El 		= 0.0;
	g_PCSOut.LOSAngCmd.dAz		= dTheta_Init_Az;
	g_PCSOut.LOSAngCmd.dEl		= dTheta_Init_El;
	g_PCSOut.LOSFFACmd.dAz		= 0.0;
	g_PCSOut.LOSFFACmd.dEl		= 0.0;
	bSlewing_Az                = FALSE;
	bSlewing_El                = FALSE;
	g_PCSOut.bSlewComplete		= TRUE;
	dT_Az 						= 0.0;
	dT_El 						= 0.0;
}

/*
**************************************************************************************************************************************************************
**
**   FUNCTION: Parabolic_Cmd_Shaping
**
**   DESCRIPTION:
**		This module generates a parabolic shaped trajectory between two angles for
**		2 axes, Azimuth and Elevation.
**		- It is intended to be called continuously.
**		- All processing is independent between Az and El.
**		- When called with a new starting or ending angle, this module
**			initializes various terms that remain constant until the trajectory
**			has reached the new ending angle.
**		- Changes in the starting or ending angle are ignored until the new
**			ending angle has been reached.
**
**   INPUTS:
**		Name							Units				Data Type 
**		g_PCSIn						n/a				Input structure 
**		g_dMjrFrameTime			Seconds			double
**		g_PCSIn.LOSInit.dAz		Radians			double
**		g_PCSIn.LOSInit.dEl		Radians			double 
**    g_PCSIn.LOSCmd.dAz		Radians			double
**    g_PCSIn.LOSCmd.dEl		Radians			double   
**		g_PCSIn.InitStart.bAz	Logical			int
**		g_PCSIn.InitStart.bEl	Logical			int
**
**   OUTPUTS:
**		None
**
**   RETURN:
**		Name							Units				Data Type  
**		g_PCSOut						n/a				Output structure
**		g_PCSOut.LOSAngCmd.dAz	Radians			double
**		g_PCSOut.LOSAngCmd.dEl	Radians			double
**		g_PCSOut.PCSFFACmd.dAz	Radians/Sec		double
**		g_PCSOut.PCSFFACmd.dEl	Radians/Sec		double  
**		g_PCSOut.bSlewComplete  Logical			int
**
**   RESTRICTIONS:
**      None
**
*******************************************************************************
**
**  Revision History
**
**  $Log: $
**
*******************************************************************************
*/
void Parabolic_Cmd_Shaping()
{   
	
	/* Initialization for a new slew */

	/* If a new Az Theta angle has been commanded */
	if( (( g_PCSIn.LOSCmd.dAz != dTheta_Cmd_Past_Az ) && !bSlewing_Az )
		||( g_PCSIn.InitStart.bAz && !bSlewing_Az ) )
	{
		/* Initialize Local Parameters - AZ */
		if ( g_PCSIn.InitStart.bAz )
		{
			dTheta_Init_Az = g_PCSIn.LOSInit.dAz;
			g_PCSIn.InitStart.bAz = FALSE;
		}
		else
		{
			dTheta_Init_Az = g_PCSOut.LOSAngCmd.dAz;
		}

		dT_Zero_Az = g_dMjrFrameTime;
		dTheta_Az = 0.0;
		dDelta_Theta_Cmd_Az = g_PCSIn.LOSCmd.dAz - dTheta_Init_Az;
		dTheta_Cmd_Past_Az = g_PCSIn.LOSCmd.dAz;

		/* If 2 Piece transition: accel and decel */
		if( fabs(dDelta_Theta_Cmd_Az) < dTheta_2Piece_Az )
		{  
			dT_Decel_Az = sqrt( fabs(dDelta_Theta_Cmd_Az) / PCS_MAX_ACCEL_AZ );
			dT_End_Az = 2 * dT_Decel_Az;
		}
		/* Else, if 3 Piece transition: accel, coast and decel */
		else
		{
			dT_Decel_Az = dT_Vmax_Az + ( fabs(dDelta_Theta_Cmd_Az)
						- dTheta_2Piece_Az ) / PCS_MAX_RATE_AZ;
			dT_End_Az = dT_Decel_Az + dT_Vmax_Az;
		}

	}

	/* If a new El Theta angle has been commanded */
	if( (( g_PCSIn.LOSCmd.dEl != dTheta_Cmd_Past_El ) && !bSlewing_El)
		||( g_PCSIn.InitStart.bEl && !bSlewing_El ) )
	{
		/* Initialize Local Parameters - EL */
		if ( g_PCSIn.InitStart.bEl )
		{
			dTheta_Init_El = g_PCSIn.LOSInit.dEl;
			g_PCSIn.InitStart.bEl = FALSE;
		}
		else
		{
			dTheta_Init_El = g_PCSOut.LOSAngCmd.dEl;
		}

		dT_Zero_El = g_dMjrFrameTime;
		dTheta_El = 0.0;
		dDelta_Theta_Cmd_El = g_PCSIn.LOSCmd.dEl - dTheta_Init_El;
		dTheta_Cmd_Past_El  = g_PCSIn.LOSCmd.dEl;

		/* If 2 Piece transition: accel and decel */
		if( fabs(dDelta_Theta_Cmd_El) < dTheta_2Piece_El )
		{
			dT_Decel_El = sqrt( fabs(dDelta_Theta_Cmd_El) / PCS_MAX_ACCEL_EL );
			dT_End_El = 2 * dT_Decel_El;
		}
		/* Else, if 3 Piece transition: accel, coast and decel */
		else
		{
			dT_Decel_El = dT_Vmax_El + ( fabs(dDelta_Theta_Cmd_El)
						- dTheta_2Piece_El ) / PCS_MAX_RATE_EL;
			dT_End_El = dT_Decel_El + dT_Vmax_El;
		}

	}


	/* Calculate parabolic trajectory outputs */

	/* If still have more AZ angle to move */
	if( fabs(dTheta_Az) < fabs(dDelta_Theta_Cmd_Az) )
	{
   		/* Set flag indicating mirror is slewing in Az */
		bSlewing_Az = TRUE;

   		/* Calculate relative time for Az slew */
		dT_Az = g_dMjrFrameTime - dT_Zero_Az;

		/* If performing 2 Piece transition */
		if( fabs(dDelta_Theta_Cmd_Az) < dTheta_2Piece_Az )
		{
			/* If in acceleration phase */
			if( dT_Az <= dT_Decel_Az )
			{
				dTheta_Az 		= ( 0.5 * PCS_MAX_ACCEL_AZ * dT_Az * dT_Az )
								* Sign(dDelta_Theta_Cmd_Az);
				g_PCSOut.LOSFFACmd.dAz = PCS_MAX_ACCEL_AZ * Sign(dDelta_Theta_Cmd_Az);
         	}
			/* Else, If in deacceleration phase */
			else if( dT_Az > dT_Decel_Az && dT_Az <= dT_End_Az )
			{
				dTheta_Az 	= dDelta_Theta_Cmd_Az - ( 0.5 * PCS_MAX_ACCEL_AZ
							* ( dT_End_Az - dT_Az ) * ( dT_End_Az - dT_Az ) )
							* Sign(dDelta_Theta_Cmd_Az);
				g_PCSOut.LOSFFACmd.dAz = -PCS_MAX_ACCEL_AZ * Sign(dDelta_Theta_Cmd_Az);
         	}
			/* Else, we are there */
			else
			{
				dTheta_Az = dDelta_Theta_Cmd_Az;
				g_PCSOut.LOSFFACmd.dAz = 0.0;
			}

      	}
		/* Else, performing 3 Piece transition */
		else
		{

			/* If in acceleration phase */
			if( dT_Az <= dT_Vmax_Az )
			{
				dTheta_Az 	= ( 0.5 * PCS_MAX_ACCEL_AZ * dT_Az * dT_Az )
							* Sign(dDelta_Theta_Cmd_Az);
				g_PCSOut.LOSFFACmd.dAz = PCS_MAX_ACCEL_AZ * Sign(dDelta_Theta_Cmd_Az);
        	}
			/* Else, If in coast phase */
			else if( dT_Az > dT_Vmax_Az && dT_Az <= dT_Decel_Az )
			{
				dTheta_Az 	= ( dTheta_Vmax_Az + PCS_MAX_RATE_AZ
							* ( dT_Az - dT_Vmax_Az ) ) * Sign(dDelta_Theta_Cmd_Az);
				g_PCSOut.LOSFFACmd.dAz = 0.0;
         	}
			/* Else, If in deacceleration phase */
			else if( dT_Az > dT_Decel_Az && dT_Az <= dT_End_Az )
			{
				dTheta_Az 	= dDelta_Theta_Cmd_Az - ( 0.5 * PCS_MAX_ACCEL_AZ
							* ( dT_End_Az - dT_Az ) *  ( dT_End_Az - dT_Az ) )
							* Sign(dDelta_Theta_Cmd_Az);
				g_PCSOut.LOSFFACmd.dAz = -PCS_MAX_ACCEL_AZ * Sign(dDelta_Theta_Cmd_Az);
         	}
			/* Else, we are there */
			else
			{
				dTheta_Az = dDelta_Theta_Cmd_Az;
				g_PCSOut.LOSFFACmd.dAz = 0.0;
			}

		}

	}
	/* Else, we have reached AZ angle */
	else
	{
		dTheta_Az = dDelta_Theta_Cmd_Az;
		g_PCSOut.LOSFFACmd.dAz = 0.0;
		bSlewing_Az = FALSE;
	}

	/* If still have more EL angle to move */
	if( fabs(dTheta_El) < fabs(dDelta_Theta_Cmd_El) )
	{
   	/* Set flag indicating mirror is slewing in El */
		bSlewing_El = TRUE;

   	/* Calculate relative time for El slew */
		dT_El = g_dMjrFrameTime - dT_Zero_El;

		/* If performing 2 Piece transition */
		if( fabs(dDelta_Theta_Cmd_El) < dTheta_2Piece_El )
		{
			/* If in acceleration phase */
			if( dT_El <= dT_Decel_El )
			{
				dTheta_El 	= ( 0.5 * PCS_MAX_ACCEL_EL * dT_El * dT_El )
							* Sign(dDelta_Theta_Cmd_El);
				g_PCSOut.LOSFFACmd.dEl = PCS_MAX_ACCEL_EL;
         	}
			/* Else, If in deacceleration phase */
			else if( dT_El > dT_Decel_El && dT_El <= dT_End_El )
			{
				dTheta_El 	= dDelta_Theta_Cmd_El - ( 0.5 * PCS_MAX_ACCEL_EL
							* ( dT_End_El - dT_El ) * ( dT_End_El - dT_El ) )
							* Sign(dDelta_Theta_Cmd_El);
				g_PCSOut.LOSFFACmd.dEl = -PCS_MAX_ACCEL_EL * Sign(dDelta_Theta_Cmd_El);
         	}
			/* Else, we are there */
			else
			{
				dTheta_El = dDelta_Theta_Cmd_El;
				g_PCSOut.LOSFFACmd.dEl = 0.0;
			}

      	}
		/* Else, performing 3 Piece transition */
		else
		{

			/* If in acceleration phase */
			if( dT_El <= dT_Vmax_El )
			{
				dTheta_El 	= ( 0.5 * PCS_MAX_ACCEL_EL * dT_El * dT_El )
							* Sign(dDelta_Theta_Cmd_El);
				g_PCSOut.LOSFFACmd.dEl = PCS_MAX_ACCEL_EL * Sign(dDelta_Theta_Cmd_El);
         	}
			/* Else, If in coast phase */
			else if( dT_El > dT_Vmax_El && dT_El <= dT_Decel_El )
			{
				dTheta_El 	= ( dTheta_Vmax_El + PCS_MAX_RATE_EL
							* ( dT_El - dT_Vmax_El ) ) * Sign(dDelta_Theta_Cmd_El);
				g_PCSOut.LOSFFACmd.dEl = 0.0;
         	}
			/* Else, If in deacceleration phase */
			else if( dT_El > dT_Decel_El && dT_El <= dT_End_El )
			{
				dTheta_El 	= dDelta_Theta_Cmd_El - ( 0.5 * PCS_MAX_ACCEL_EL
							* ( dT_End_El - dT_El ) * ( dT_End_El - dT_El ) )
							* Sign(dDelta_Theta_Cmd_El);
				g_PCSOut.LOSFFACmd.dEl = -PCS_MAX_ACCEL_EL * Sign(dDelta_Theta_Cmd_El);
         	}
			/* Else, we are there */
			else
			{
				dTheta_El = dDelta_Theta_Cmd_El;
				g_PCSOut.LOSFFACmd.dEl = 0.0;
			}

		}

	}
	/* Else, we have reached EL angle */
	else
	{
		dTheta_El = dDelta_Theta_Cmd_El;
		g_PCSOut.LOSFFACmd.dEl = 0.0;
		bSlewing_El = FALSE;
	}

	/* Update new increment in angle */
	g_PCSOut.LOSAngCmd.dAz = dTheta_Init_Az + dTheta_Az;
	g_PCSOut.LOSAngCmd.dEl = dTheta_Init_El + dTheta_El;

	/* If slew is turned off in both AZ and EL, then slewing is complete*/
	if( bSlewing_Az == FALSE && bSlewing_El == FALSE )
	{
		g_PCSOut.bSlewComplete =  TRUE;	/* Set slew complete flag */
    }
	/* Else, slewing is not complete */
	else
	{
		g_PCSOut.bSlewComplete = FALSE;
	}

}

/*
*********************************************************************************
**   FUNCTION: Sign
**
**   DESCRIPTION:
**		This function determines the sign of a long float input argument.
**
**   INPUTS:
**		Name					Units					Data Type
**		dValue	 				N/A					Double
**
**   OUTPUTS:
**		None
**
**   RETURN:
**		Name					Units					Data Type
**		nSign					N/A						Integer
**
**   RESTRICTIONS:
**      None
**
*******************************************************************************
**
**  Revision History
**
**  $Log: $
**
*******************************************************************************
*/
int Sign(double dValue)
{
	int nSign = 0;

	if( dValue > 0 )
	{
		nSign = 1;
	}
	else if( dValue < 0 )
	{
		nSign = -1;
	}
	else
	{
		nSign = 0;
	}

	return nSign;
}
