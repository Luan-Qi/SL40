#include "sl40.h"
#include "time32.h"
#include "button.h"
#include "math.h"

#define sl40_LED_tmr_pr_max 999

// 冷光色温6500K，暖光色温3000K
#define COLD_TEMP 6500.0f
#define WARM_TEMP 3000.0f

// 亮度补偿系数（根据实际光通量测量调整，冷光光效/暖光光效）
#define LUMINANCE_RATIO 2.0f

SL40_ADC_Data_t sl40_adc_data;
extern uint16_t adc_buff[ADC_BUFF_SIZE];

uint8_t sl40_CW_state = 0;
uint8_t sl40_WW_state = 0;

void sl40_CW_enable(confirm_state new_state)
{
	if(new_state == TRUE){gpio_bits_set(GPIOA, GPIO_PINS_0);sl40_CW_state = 1;}
  else if(new_state == FALSE){gpio_bits_reset(GPIOA, GPIO_PINS_0);sl40_CW_state = 0;}
}

void sl40_WW_enable(confirm_state new_state)
{
	if(new_state == TRUE){gpio_bits_set(GPIOA, GPIO_PINS_1);sl40_WW_state = 1;}
  else if(new_state == FALSE){gpio_bits_reset(GPIOA, GPIO_PINS_1);sl40_WW_state = 0;}
}

//setting from 1 to 1000, 0 means turnning off
void sl40_CW_set(uint16_t new_state)
{
	if(new_state>sl40_LED_tmr_pr_max+1) return;
	if(new_state==0){tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_2, sl40_LED_tmr_pr_max);return;}
	if(sl40_CW_state==0) tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_2, sl40_LED_tmr_pr_max);
	else tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_2, (sl40_LED_tmr_pr_max + 1) - new_state);
}

//setting from 1 to 1000, 0 means turnning off
void sl40_WW_set(uint16_t new_state)
{
	if(new_state>sl40_LED_tmr_pr_max+1) return;
	if(new_state==0){tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_1, sl40_LED_tmr_pr_max);return;}
	if(sl40_WW_state==0) tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_1, sl40_LED_tmr_pr_max);
	else tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_1, (sl40_LED_tmr_pr_max + 1) - new_state);
}


#define ADC_MAX        4095.0f
#define PWM_MAX        777.0f

#define DEAD_ZONE      0.0f
#define FILTER_FAST    0.6f
#define FILTER_SLOW    0.2f

#define GAMMA          2.2f
#define CCT_CURVE      1.5f

#define SMOOTH_STEP    0.2f
#define MIN_BRIGHT     0.01f

// ================= 状态变量 =================
static float cct_fast = 0;
static float cct_slow = 0;
static float dim_fast = 0;
static float dim_slow = 0;

static float cw_out = 0;
static float ww_out = 0;

// ================= 工具函数 =================

static float low_pass(float input, float *state, float alpha)
{
    *state = (*state) * (1.0f - alpha) + input * alpha;
    return *state;
}

static float dual_filter(float input, float *fast, float *slow)
{
    float f;
    float s;

    f = low_pass(input, fast, FILTER_FAST);
    s = low_pass(input, slow, FILTER_SLOW);

    return 0.7f * f + 0.3f * s;
}

static float apply_deadzone(float adc)
{
    float range;

    if (adc < DEAD_ZONE)
        return 0.0f;

    adc = adc - DEAD_ZONE;
    range = ADC_MAX - DEAD_ZONE;

    if (range <= 0.0f)
        return 0.0f;

    return adc / range;
}

static float smooth_transition(float current, float target)
{
    return current + (target - current) * SMOOTH_STEP;
}

// ================= 主控制函数 =================

void sl40_light_control_advanced(uint16_t cct_adc_in, uint16_t dim_adc_in)
{
    float cct_adc;
    float dim_adc;

    float ratio;
    float brightness;

    float cw_ratio;
    float ww_ratio;
    float norm;

    float cw_target;
    float ww_target;

    uint16_t cw_pwm;
    uint16_t ww_pwm;

    // 1.读取ADC
    cct_adc = (float)cct_adc_in;
    dim_adc = (float)dim_adc_in;

    // 2.双滤波
    cct_adc = dual_filter(cct_adc, &cct_fast, &cct_slow);
    dim_adc = dual_filter(dim_adc, &dim_fast, &dim_slow);

    // 3.归一化
    ratio = cct_adc / ADC_MAX;
    brightness = apply_deadzone(dim_adc);

    // 4.亮度保护
    if (brightness < MIN_BRIGHT)
    {
        cw_out = 0;
        ww_out = 0;

        sl40_CW_set(0);
        sl40_WW_set(0);
        return;
    }

    // 5.Gamma校正
    brightness = powf(brightness, GAMMA);

    // 6.色温曲线修正
    ratio = powf(ratio, CCT_CURVE);

    // 7.功率归一化
    cw_ratio = ratio;
    ww_ratio = 1.0f - ratio;

    norm = sqrtf(cw_ratio * cw_ratio + ww_ratio * ww_ratio);
    if (norm < 1e-6f)
        norm = 1.0f;

    cw_ratio = cw_ratio / norm;
    ww_ratio = ww_ratio / norm;

    // 8.目标值
    cw_target = brightness * cw_ratio;
    ww_target = brightness * ww_ratio;

    // 9.平滑
    cw_out = smooth_transition(cw_out, cw_target);
    ww_out = smooth_transition(ww_out, ww_target);

    // 10.PWM转换
    cw_pwm = (uint16_t)(cw_out * PWM_MAX);
    ww_pwm = (uint16_t)(ww_out * PWM_MAX);

    // 11.输出
    sl40_CW_set(cw_pwm);
    sl40_WW_set(ww_pwm);
}

void sl40_adc_push(void)
{
	memcpy(&sl40_adc_data, adc_buff, sizeof(sl40_adc_data));
}

// 实现gpio_bits_toggle函数
void gpio_bits_toggle(gpio_type *gpio_x, uint16_t pins)
{
    flag_status current_state = gpio_output_data_bit_read(gpio_x, pins);
    
    if (current_state == RESET) {
        gpio_bits_set(gpio_x, pins); // 如果当前状态是RESET，则设置为SET
    } else {
        gpio_bits_reset(gpio_x, pins); // 如果当前状态是SET，则重置为RESET
    }
}

uint8_t sl40_test_state = 0;

void sl40_test_shift(void)
{
	sl40_test_state++;
	if(sl40_test_state>1) sl40_test_state = 0;
	switch(sl40_test_state)
	{
		case 0:
			sl40_CW_enable(FALSE);
			sl40_WW_enable(FALSE);
			gpio_bits_set(GPIOF, GPIO_PINS_1);
			break;
		case 1:
			sl40_CW_enable(TRUE);
			sl40_WW_enable(TRUE);
			gpio_bits_reset(GPIOF, GPIO_PINS_1);
			break;	
	}
}


#define BATTERY_LOW_THRESHOLD 2480 // 假设电池电压低于3000mV时认为电量低，这个值你可以根据实际情况调整
#define BLINK_INTERVAL 5 // 20Hz的循环中需要循环20次达到1秒的闪烁间隔
#define INIT_ADC_READS 5 // 开机时进行5次ADC读取

uint32_t sl40_time = 0;
uint8_t blink_counter = 0; // 用于控制闪烁间隔的计数器
uint8_t restart_flag = 0;
uint8_t init_adc_counter = 0; // 用于控制初始化ADC读取次数的计数器
uint8_t is_initialized = 0; // 开机初始化标志

void sl40_main_run(void)
{
	if(millis_overstep(sl40_time)) sl40_time = millis() + 50;
	else return;
	
	sl40_adc_push();
	if (!is_initialized)
	{
		if (init_adc_counter < INIT_ADC_READS)
		{
				init_adc_counter++;
		}
		else
		{
				is_initialized = 1; // 初始化完成
		}
		return;
	}
	
	if(sl40_adc_data.vbat_adc < BATTERY_LOW_THRESHOLD || restart_flag)
	{
		// 电池电压低，使用固定值
		sl40_light_control_advanced(0, 0);
		// 控制GPIO闪烁
		if(blink_counter >= BLINK_INTERVAL)
		{
			gpio_bits_toggle(GPIOF, GPIO_PINS_1); // 切换GPIO状态以实现闪烁
			blink_counter = 0; // 重置计数器
		}
		else
		{
			blink_counter++; // 增加计数器
		}
		restart_flag = 1;
	}
	else
	{
		// 电池电压正常，使用ADC读取的值
		sl40_light_control_advanced(sl40_adc_data.CCT_value_adc, 4096 - sl40_adc_data.DIM_value_adc);
	}
}


void sl40_main(void)
{
	sl40_main_run();
	button_main_run();
}



