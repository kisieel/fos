#include "stm32l1xx.h"

// Maximum number of colors available to set
#define led_colors_qnt  6

#define led_length			5
#define led_bits				24
#define led_select_all	200

#define led_low 	12
#define led_high 	30
#define brake			0xFF

#define led_reset_state 10
#define led_on_state		20

#define color_red			16
#define color_green		8
#define color_blue		0

#define led_increment		10
#define led_decrement		20

#define animate_mode_1	10	// kazda z diod zapala sie osobno 
#define animate_mode_2	20	// kazda z diod gasi sie osobno 
#define animate_mode_3	30	// wszystkie razem przyciemniaja  sie i rozjasniaja
#define animate_mode_4	40	// 3 razy mrugniecie
#define	animate_mode_off	99	// off
#define animate_mode_delay	255	// 300ms delay

#define led_brightness_max	10
#define led_brightness_min	20

#define led_limit_max 0x79


#define led_blink_on 		10
#define led_blink_off		20

extern volatile uint16_t led_value[led_bits];
extern volatile uint16_t led[led_length*led_bits];
extern volatile	uint32_t led_state[led_length];
extern volatile uint8_t animate_mode[2];	//current[0], old [1]

extern volatile uint32_t led_blink_led_state;
extern volatile uint8_t led_blink_status;

extern volatile uint8_t	led_brightness_coeff;

extern volatile uint8_t _LED_dma_flag;
extern volatile uint8_t _LED_refresh_flag;

void led_DMA_init(uint16_t mode);
void led_timer_init(void);
void led_gpio_init(void);
void led_timer_on(void);
void led_bus_reset(void);
void led_refresh_timer_init(void);
void LED_set_values(uint8_t led_number, uint8_t blue_set, uint8_t red_set, uint8_t green_set);

void _LED_init(void);
void _LED_set(void);
void _LED_on(void);
void _LED_off(void);
void _LED_set_color(uint8_t led_number, uint8_t blue, uint8_t red, uint8_t green);
void _LED_set_color_list(uint8_t led_number, uint8_t index);
uint8_t _LED_change_color(uint8_t led_number, uint8_t color, uint8_t step, uint8_t direction);
uint32_t _LED_change_brightness(uint8_t led_number, uint8_t step, uint8_t direction);
uint32_t _LED_change_brightness_perc(uint8_t led_n, uint8_t perc);
uint32_t _LED_change_brightness_all(uint8_t step, uint8_t direction);
uint32_t _LED_change_brightness_all_perc(uint8_t perc);

void _LED_change_brightness_limit(uint8_t brightness);

void _LED_blink_on(uint8_t led_number); 
void _LED_blink_off(uint8_t led_number); 

void _LED_refresh(uint16_t delay_ms);
void _LED_animate(void);

uint8_t reverse(uint8_t b);
void wait_(uint32_t time);