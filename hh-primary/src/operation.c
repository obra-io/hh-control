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

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief
  * @param  none
  * @retval
  */
void operation_loop(void)
{
	static Alesi_HandleTypeDef digital_io_h;

	if (digital_io_h == 0U)
	{
		digital_io_h = alesi_resolve_uri(":digital_io");
	}
	else
	{
		uint8_t in[2];

		/* Echo inputs */
		alesi_read(digital_io_h, in, sizeof(in));

		in[0] = 0x55;

		alesi_write(digital_io_h, in, 1);
	}
}

/*****************************END OF FILE************************************/
