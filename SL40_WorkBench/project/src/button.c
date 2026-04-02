#include "button.h"
#include "time32.h"
#include "sl40.h"

uint32_t button_time = 0;
uint8_t button_down = 0;
uint16_t button_down_long = 0;

void button_main_run(void)
{
	if(millis_overstep(button_time)) button_time = millis() + 10;
	else return;
	
	if(button_down==0)
	{
		if(BUTTON0==RESET)
		{
			button_down = 1;
		}
	}
	else
	{
		button_down_long++;
		if(button_down_long>500)
		{
			button_down_long = 0;
			return;
		}
		if(BUTTON0==SET)
		{
			sl40_test_shift();
			button_down = 0;
			button_down_long = 0;
		}
	}
}

