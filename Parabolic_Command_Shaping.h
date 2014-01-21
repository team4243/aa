/* Parabolic_Cmd_Shaping.h */ 
 
#include "defines.h"

/* Logical flags */
typedef struct
{
	int bAz;
	int bEl;
} Log_Ang_Flags_Struct;  

/* Input data structure */ 
typedef struct
{                    
	Angle_Dbl_Struct		LOSInit;
	Angle_Dbl_Struct		LOSCmd;
	Log_Ang_Flags_Struct	InitStart;
} PCS_Data_In_Struct;

/* Output data structure */
typedef struct
{
	Angle_Dbl_Struct		LOSAngCmd;
	Angle_Dbl_Struct		LOSFFACmd;
	int						bSlewComplete;
} PCS_Data_Out_Struct;

/* External references */
extern PCS_Data_In_Struct  g_PCSIn;
extern PCS_Data_Out_Struct g_PCSOut;
