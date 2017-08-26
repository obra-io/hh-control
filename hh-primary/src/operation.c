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
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static enum States {
	INITIALIZING,
	PRE_FLIGHT,
	PUSHER_INTERLOCK,
	PUSHING,
	COASTING,
	BRAKING,
	EMERGENCY_BRAKING,
	STOPPED
}Current_State =INITIALIZING, Next_State;

static Alesi_HandleTypeDef digital_io_h;
static Alesi_HandleTypeDef l_height1_h;
static Alesi_HandleTypeDef l_height2_h;
static Alesi_HandleTypeDef l_height3_h;
static Alesi_HandleTypeDef l_height4_h;
static Alesi_HandleTypeDef lat1_h;
static Alesi_HandleTypeDef lat2_h;
static Alesi_HandleTypeDef rpi_h;
static Alesi_HandleTypeDef bms1_h ;
static Alesi_HandleTypeDef pusher_h;
static Alesi_HandleTypeDef Toggle_relay_detect_secondary_h;
static Alesi_HandleTypeDef Ground_control_coomunication_h;
static Alesi_HandleTypeDef Current_State_h;
static Alesi_HandleTypeDef Safety_Critical_situtaions_h;
static Alesi_HandleTypeDef pitch_yaw_roll_h;
static Alesi_HandleTypeDef levitation_height_h;
static Alesi_HandleTypeDef Kinematics_h;
static Alesi_HandleTypeDef sensor_failure_h;
static Alesi_HandleTypeDef telemetry_h;

static Alesi_Timestamped8TypeDef *l_height1;
static Alesi_Timestamped8TypeDef *l_height2;
static Alesi_Timestamped8TypeDef *l_height3;
static Alesi_Timestamped8TypeDef *l_height4;
static Alesi_Timestamped8TypeDef *lat1;
static Alesi_Timestamped8TypeDef *lat2;
static Alesi_Timestamped8TypeDef *rpi;
static Alesi_Timestamped8TypeDef *bms1;
static Alesi_Timestamped8TypeDef *Toggle_relay_detect_secondary;


static bool isInit_l_height1 = false; //Lev front right
static bool isInit_l_height2 = false; //Lev back left
static bool isInit_l_height3 = false; //Lateral back right
static bool isInit_l_height4 = false; //Lateral front left
static bool isInit_lat1 = false; //Front Lateral
static bool isInit_lat2 = false; //Back Lateral.. I think..
static bool isInit_rpi = false ; //Front Lateral
static bool isInit_bm1 = false ; //Front Lateral


/* Private function prototypes -----------------------------------------------*/
static void current_state_initilization_handler(void);
static void current_state_pre_flight_handler(void);
static void current_state_pusher_interlock_handler(void);
static void current_state_pushing_handler(void);
static void current_state_coasting_handler(void);
static void current_state_braking_handler(void);
static void current_state_emergency_braking_handler(void);
static void current_state_stopped_handler(void);

static int timer_get_count(void);
static void free_running_timer_init(void);
static void gpio_rr_init_frnt_rear(void);

/* Private functions ---------------------------------------------------------*/

static bool resolve_resources(void)
{
	bool all_resolved = false;

	if (!digital_io_h)
	{
		digital_io_h = alesi_resolve_uri(":digital_io");
	}

	if (!l_height1_h)
	{
		l_height1_h = alesi_resolve_uri(":l_height1");
	}

	if (!l_height2_h)
	{
		l_height2_h = alesi_resolve_uri(":l_height2");
	}

	if (!l_height3_h)
	{
		l_height3_h = alesi_resolve_uri(":l_height3");
	}

	if (!l_height4_h)
	{
		l_height4_h = alesi_resolve_uri(":l_height4");
	}

	if (!lat1_h)
	{
		lat1_h = alesi_resolve_uri(":lat1");
	}

	if (!lat2_h)
	{
		lat2_h = alesi_resolve_uri(":lat2");
	}

	if (!rpi_h)
	{
		rpi_h = alesi_resolve_uri(":rpi");
	}
	if(!Toggle_relay_detect_secondary_h)
	{
		Toggle_relay_detect_secondary_h=alesi_resolve_uri(":Toggle_relay_detect_secondary_h");
	}
	if(!bms1_h)
	{
		bms1_h=alesi_resolve_uri(":bms1");
	}
	if (digital_io_h && l_height1_h && l_height2_h && l_height3_h && l_height4_h && lat1_h \
			&& lat2_h && rpi_h &&Toggle_relay_detect_secondary_h && bms1)
	{
		all_resolved = true;
	}

	return all_resolved;
}

static void run_state_machine(void)
{

	switch(Current_State)
	{
	case INITIALIZING:

		current_state_initilization_handler();
		break;
	case PRE_FLIGHT:
		current_state_pre_flight_handler();
		break;
	case PUSHER_INTERLOCK:
		current_state_pusher_interlock_handler();
		break;
	case PUSHING:
		current_state_pushing_handler();
		break;
	case COASTING:
		current_state_coasting_handler();
		break;
	case BRAKING:
		current_state_braking_handler();
		break;
	case EMERGENCY_BRAKING:
		current_state_emergency_braking_handler();
		break;
	case STOPPED:
		current_state_stopped_handler();
		break;
	default:
		break;
	}
	Current_State= Next_State;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief
  * @param  none
  * @retval
  */
void operation_loop(void)
{
	resolve_resources();
	//initialize the hardware
	gpio_rr_init_frnt_rear(); //init retro reflector
	//timer_get_count(); //useless, refactor
	free_running_timer_init();

	l_height1 = alesi_resource_from_h(l_height1_h, Alesi_Timestamped8TypeDef);
	l_height2 = alesi_resource_from_h(l_height2_h, Alesi_Timestamped8TypeDef);
	l_height3 = alesi_resource_from_h(l_height3_h, Alesi_Timestamped8TypeDef);
	l_height4 = alesi_resource_from_h(l_height4_h, Alesi_Timestamped8TypeDef);
	bms1 = alesi_resource_from_h(bms1_h, Alesi_Timestamped8TypeDef);
	lat1 = alesi_resource_from_h(lat1_h, Alesi_Timestamped8TypeDef);
	lat2 = alesi_resource_from_h(lat2_h, Alesi_Timestamped8TypeDef);
	rpi = alesi_resource_from_h(rpi_h, Alesi_Timestamped8TypeDef);
	Toggle_relay_detect_secondary = alesi_resource_from_h(Toggle_relay_detect_secondary_h, Alesi_Timestamped8TypeDef);

	if (l_height1 != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - l_height1->stamp) < 100U)
		{
			uint8_t lev_height_frnt_rt[8];
			//if latest time stamp, pull value
			//alesi_read(l_height1_h, lev_height_frnt_rt, sizeof (lev_height_frnt_rt));
			memcpy(l_height1->data,&lev_height_frnt_rt,8);
			if(lev_height_frnt_rt<=0x4096)
			{
				isInit_l_height1 = true;
			}

	}

	if (l_height2 != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - l_height2->stamp) < 50U)
		{
			uint8_t lev_height_frnt_left[8];
			//if latest time stamp, pull value
			memcpy(l_height2->data,&lev_height_frnt_left,8);
			if(lev_height_frnt_left<=0x4096)
			{
				isInit_l_height2 = true;
			}
		}

	if (l_height3 != NULL)
		{
			//check if the message is latest, if so, read the value
			if ((alesi_get_tick_count() - l_height3->stamp) < 50U)
			{
				uint8_t lev_height_bk_left[8];
				//if latest time stamp, pull value
			memcpy(l_height3->data,&lev_height_bk_left,8);
			if(lev_height_bk_left<=0x4096)
			{
				isInit_l_height3 = true;
			}
		}


	if (l_height4 != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - l_height4->stamp) < 50U)
		{
			uint16_t lev_height_bk_right[2];
			//if latest time stamp, pull value
			alesi_read(l_height4_h, lev_height_bk_right, sizeof (lev_height_bk_right));
		}

		isInit_l_height4 = true;
	}

	if (lat1 != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - lat1->stamp) < 50U)
		{
			uint16_t lat_height1[8];
			//if latest time stamp, pull value
			alesi_read(lat1_h, lat_height1, sizeof(lat_height1));
		}
		isInit_lat1 = true;
	}

	if (lat2 != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - lat2->stamp) < 50U)
		{
			uint8_t lat_height2[2];
			//if latest time stamp, pull value
			alesi_read(lat2_h, lat_height2, sizeof(lat_height2));
		}
		isInit_lat2 = true;
	}

	if (rpi != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - rpi->stamp) < 50U)
		{
			uint8_t rpi_msg[8];
			//if latest time stamp, pull value
			alesi_read(rpi_h, rpi_msg, sizeof(rpi));
		}
		isInit_rpi = true;
	}

	if (bms1 != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - rpi->stamp) < 50U)
		{
			uint8_t bms1_msg[8];
			//if latest time stamp, pull value
			alesi_read(bms1_h, bms1_msg, sizeof(bms1));
		}
		isInit_bm1 = true;
	}




	if (resolve_resources())
	{
		run_state_machine();
	}
}
//--------------------- current_state_initilization-----------------------//
static void current_state_initilization_handler()
{
	//check all the sensors, make sure we are reading values, get ready for pre flight


	//TOGGLE RELAYS
//	uint8_t in_relay[2];
//	/* Echo inputs */
//	alesi_read(digital_io_h, in_relay, sizeof(in_relay));
//	if(in_relay[0]== 0x0f)//location of the relay from secondary controller wokring)
//	{
//		alesi_write(Toggle_relay_detect_secondary_h,in_relay,1);
//	}
//	in_relay[1] = 0xf;
//	alesi_write(Toggle_relay_detect_secondary_h,in_relay,1);


	//If we have resolved every sensor, move into pre flight mode
	if (isInit_l_height1 && isInit_l_height2 && isInit_l_height3 && isInit_l_height4 && isInit_lat1 \
			&& isInit_lat1 && isInit_rpi == true)
	{
		Next_State = PRE_FLIGHT;
	}
}
//--------------------- current_state_pre_flight-----------------------//
static void current_state_pre_flight_handler()
{
	Alesi_Timestamped8TypeDef * pusher;
	pusher = alesi_resource_from_h(pusher_h, Alesi_Timestamped8TypeDef);
	if (pusher != NULL)
	{
		//check if the message is latest, if so, read the value
		if ((alesi_get_tick_count() - pusher->stamp) < 50U)
		{
			uint8_t pusher_msg[8];
			//if latest time stamp, pull value
			alesi_read(pusher_h, pusher_msg, sizeof (pusher));

			//if pusher message is true, change the state to pusher engaged
			if(true)
			{
				uint8_t pusher_res[1];
				alesi_write(pusher_h, pusher_res, sizeof (pusher));
			}
		}
	}
}
//--------------------- current_state_pusher_interlock-----------------------//
static void current_state_pusher_interlock_handler()
{

}
//--------------------- current_state_pushing-----------------------//
static void current_state_pushing_handler()
{

}
//--------------------- current_state_ coasting-----------------------//
static void current_state_coasting_handler()
{

}
//--------------------- current_state_ braking-----------------------//
static void current_state_braking_handler()
{

}
//--------------------- current_state_ emergency stop-----------------------//
static void current_state_emergency_braking_handler()
{

}
//--------------------- current_state_ stopped handler-----------------------//
static void current_state_stopped_handler()
{

}

//initialize GPIO
static void gpio_rr_init_frnt_rear(void)
{
//	rr1- in5
//	rr3- in6
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();


	GPIO_InitTypeDef gpio_rr_init;

	gpio_rr_init.Pin  =GPIO_PIN_2;			//pd2 digital in 6
	gpio_rr_init.Mode =GPIO_MODE_IT_RISING_FALLING;
	gpio_rr_init.Pull =GPIO_PULLDOWN;
	gpio_rr_init.Alternate =GPIO_MODE_INPUT;
	gpio_rr_init.Speed =GPIO_SPEED_FREQ_VERY_HIGH;
	//HAL_GPIO_Init(GPIOD,gpio_rr_init);

	gpio_rr_init.Pin  =GPIO_PIN_12;			//pc12 Digital in 5
	//HAL_GPIO_Init(GPIOC,gpio_rr_init);

	//interrupts
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void EXTI2_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(2);
}

void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(12);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
	uint32_t rr1_timestamp[2];
	uint32_t rr3_timestamp[2];

	if (GPIO_pin == 2)
	{
		if (GPIO_pin == SET)
		{
			rr1_timestamp[1] = rr1_timestamp[0];
			rr1_timestamp[0] = timer_get_count();
		}
	}
	else if (GPIO_pin == 12)
	{
		if (GPIO_pin == SET)
		{
			rr3_timestamp[1] = rr3_timestamp[0];
			rr3_timestamp[0] = timer_get_count();
		}
	}
}

static int timer_get_count(void)
{
	uint32_t timer_count; //this doesnt get set?

	return timer_count;
}

static TIM_HandleTypeDef timer_instance;

static void free_running_timer_init()
{
	__HAL_RCC_TIM2_CLK_ENABLE();
	timer_instance.Init.Prescaler = 4;
	timer_instance.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer_instance.Init.Period = 0xFFFFFFF;
	timer_instance.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timer_instance.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&timer_instance);
	HAL_TIM_Base_Start(&timer_instance);
}
/*
 *
 * uint32_t rr3_timestamp[2];
 *
 * if (rr1[0] > rr[1])
 * {
 * 		delta = rr1[0] - rr[1];
 * }
 * else
 * {
 * 		delta = rr1[0] + (UINT32_MAX - rr1[1]);
 * }
 *
 *
 *
 *
 */


/*****************************END OF FILE************************************/
