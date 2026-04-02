#ifndef __BUTTON__
#define __BUTTON__

#include "at32f421.h"

#define BUTTON0_GPIO				GPIOA
#define BUTTON0_PIN					GPIO_PINS_10

#define BUTTON0 		gpio_input_data_bit_read(BUTTON0_GPIO, BUTTON0_PIN)

#ifdef __cplusplus
 extern "C" {
#endif

void button_main_run(void);
	 
#ifdef __cplusplus
}
#endif

#endif

