/**
  ******************************************************************************
  * @file    operation.c
  * @author  Brandon Ortiz
  * @version V1.0.0
  * @date    15-May-2017
  * @brief
  *
  */

/* Includes ------------------------------------------------------------------*/
#include <alesi.h>

#include "operation.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
enum States {
	INITIALIZING,
	PRE_FLIGHT,
	PUSHER_INTERLOCK,
	PUSHING,
	COASTING,
	BRAKING,
	EMERGENCY_BRAKING,
	STOPPED
}CS=INITIALIZING;

static Alesi_HandleTypeDef digital_io_h;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static bool resolve_resources(void)
{
	bool all_resolved = false;

	if (!digital_io_h)
	{
		digital_io_h = alesi_resolve_uri(":digital_io");
	}

	if (digital_io_h)
	{
		all_resolved = true;
	}

	return all_resolved;
}

static void run_state_machine(void)
{
	switch(CS)
	{
/*	case INITIALIZING:
		in[0] = 0x0f; //toggling relays
		alesi_write(digital_io_h, in, 1);

		//read_can messages
		//can message
		if(can_id=0x091) //can messages stored in the dict
			{
				//levitation height sensor front right
			}
		if(can_id=0x092)
			{
				//levitation height sensor front right
			}
		if(can_id=0x093)
			{

			}

		//toggle the relays, drive an output on all 6
		//watch inputs, check if been set
		if( alesi_read(digital_io_h,in,sizeof(in))== 0x0f)
					{
						relay_flag_working =1;
					}
		break;
	case PRE_FLIGHT:

		break;
	case PUSHER_INTERLOCK:
		break;
	case PUSHING:
		break;
	case COASTING:
		break;
	case BRAKING:
		break;
	case EMERGENCY_BRAKING:
		break;*/
	}

}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief
  * @param  none
  * @retval
  */
void operation_loop(void)
{

	uint8_t in[2];
	uint8_t relay_flag_working; //relay flag
	uint8_t lh_fr_flg; //lev front right
	uint8_t lh_rl_flg; //lev rear left
	uint8_t sn_prsh_flg;
	uint8_t bms_flg;  //b,s flag
	uint8_t lat_rr_stb_flg; //lat rear stability flag
	uint8_t rf_rr_flg; //fight front retro
	uint8_t rr_rr_flg; //right rear retro reflector

	//what does this do?
	if (resolve_resources())
	{
		run_state_machine();
	}
}

/*****************************END OF FILE************************************/
