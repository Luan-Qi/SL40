#ifndef __SL40_H
#define __SL40_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f421_wk_config.h"
#include "string.h"
	
typedef struct {
	uint16_t CCT_value_adc;
	uint16_t DIM_value_adc;
	uint16_t ics_adc;
	uint16_t vbat_adc;
	uint16_t ntc_adc;
}SL40_ADC_Data_t;

void sl40_test_shift(void);
void sl40_main(void);

#ifdef __cplusplus
}
#endif

#endif
