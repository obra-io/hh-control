/**
  ******************************************************************************
  * @file    main.c
  * @author  Brandon Ortiz
  * @version V1.0.0
  * @date    24-May-2017
  * @brief
  ******************************************************************************
  * @attention
  *
  * This program is open source software: you can redistribute it and/or
  * modify it under the terms of the GNU General Public License as published
  * by the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
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
  *
  */
int main(void)
{
    Alesi_StatusTypeDef status;

     status = alesi_init();

    if (status == ALESI_OK)
	{
    	Alesi_HandleTypeDef user_loop_h;

    	user_loop_h = alesi_resolve_uri(":actor:user_loop");

    	status = alesi_reg_loop(user_loop_h, operation_loop);
	}

    if (status == ALESI_OK)
    {
        alesi_start();
    }

    while(1);

    return 0;
}

void vApplicationIdleHook(void)
{
    static uint32_t i;

    i++;
}

/***************************************************************END OF FILE****/
