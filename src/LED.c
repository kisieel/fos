#include "led.h"
#include "stm32l1xx.h"

volatile uint8_t i = 0;
volatile uint16_t temp_value[led_bits];
volatile uint16_t led[led_length*led_bits];
volatile uint32_t led_state[led_length];

/* for animation */
volatile uint32_t led_state_temp[led_length];
volatile uint8_t	led_current_animation;
volatile uint8_t	animation_step;
volatile uint8_t	led_animate_brightness;
volatile uint8_t	led_brightness_temp;
/* ---------- */

volatile uint8_t LED_blinking_state;
volatile uint8_t led_blink_status[led_length];

volatile uint8_t	led_brightness_coeff;

//volatile uint8_t led_animations[led_amimations_qnt] = {
//	
//};

// Blue, Red, Green
volatile uint8_t led_colors[led_colors_qnt][3] = {
	{100, 0, 0},     // Niebieski
	{0, 100, 0},     // Czerwony
	{0, 0, 100},     // Zielony
	{0, 100, 100},   // Zolty
	{100, 55, 0},   // Fioletowy
	{100, 100, 0},    // HOW KNOWS?!
	{100, 0, 100}    // 
};

volatile uint8_t led_brightness[led_brightness_qnt] = {
	5,
	15,
	25,
	35,
	55,
	75,
	100
};

volatile uint8_t _LED_dma_flag;
volatile uint8_t _LED_refresh_flag;

void led_gpio_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                // Clock for GPIOB 
	
	GPIOB->MODER |= (GPIO_MODER_MODER10_1);             // Alternate function 
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_10 );              // Output push-pull
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1;      	//  10 MHz Medium speed
	GPIOB->PUPDR  |=  GPIO_PUPDR_PUPDR10_0;
	
	GPIOB->AFR[1] |= (GPIO_AFRH_AFRH10 & (1 << 4*2));
}

void led_timer_init(void)
{
	/*      Tim2 ch3   */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 	
	TIM2->CR1 &= ~(TIM_CR1_CKD 	//clock division - none
						| TIM_CR1_ARPE			//TIMx_ARR register is not buffered
						| TIM_CR1_CMS		//Edge-aligned mode
						| TIM_CR1_DIR		// up counter
						| TIM_CR1_UDIS		//UEV enabled
						);
	//TIM2->CR1 |=   TIM_CR1_CMS_1  ;		// one pulse mode
	
	TIM2->CR2 |= TIM_CR2_CCDS;
	
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;    // OC3, PWM mode 1 , upcounting
 
 	TIM2->CCER |= TIM_CCER_CC3E ;                          // OC3 enable
	
	TIM2->DIER |=   TIM_DIER_UDE ;
	TIM2->PSC =  0;// prescaler value fCK_PSC / (PSC[15:0] + 1)		50ns tick
	TIM2->ARR = 39;			//	auto-reload value <--- 1250ns period
	//TIM2->CCR3 = 0;		//	duty cycle
	
	TIM2->EGR |= TIM_EGR_UG;                               // Initialize all registers
	TIM2->SR &= ~ TIM_SR_UIF;				// czyszczenie flagi
	
}

void led_timer_on(void)
{
	_LED_dma_flag = 1;
	TIM2->CR1 |= TIM_CR1_CEN;	
	while (_LED_dma_flag){};
}

void led_bus_reset(void)
{
	uint8_t i = 0;
	for(i = 0; i< 24; i++)
	{
		temp_value[i] = 0x00;
	}
	_LED_dma_flag = 1;
		TIM2->CR1 |= TIM_CR1_CEN;	
		while (_LED_dma_flag){};
}


void led_DMA_init(uint16_t mode)
{
	TIM2->CR2 |= TIM_CR2_CCDS;                 // DMA access enable
	
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;         // Clock for DMA

	DMA1_Channel2->CCR |= DMA_CCR1_MINC       // Memory increment ENABLE
										 | DMA_CCR1_PSIZE_0       // Peripheral 32-bit size
										 | DMA_CCR1_MSIZE_0       // Memory 32-bit size
										 | DMA_CCR1_PL          // Priority: very high
										 | DMA_CCR1_CIRC;       // Circular mode
	DMA1_Channel2->CCR &= ~( DMA_CCR1_PINC );      // Peripheral increment mode DISABLE
	DMA1_Channel2->CCR |= DMA_CCR1_DIR; 		   // Write to peripheral
										 
	DMA1_Channel2->CPAR = (unsigned int)0x4000003C;      // Peripheral adress on DR in ADC1

	if(mode == led_reset_state)
	{
		DMA1_Channel2->CCR &= ~(DMA_CCR1_EN | DMA_CCR1_TCIE);   
		DMA1_Channel2->CMAR = (unsigned int)&temp_value;        // Memory adress
		DMA1_Channel2->CNDTR = led_bits;                 // 8 data to transfer (8 ADC channels)
	}
	else if(mode == led_on_state)
	{
		DMA1_Channel2->CCR &= ~(DMA_CCR1_EN | DMA_CCR1_TCIE);  
		DMA1_Channel2->CMAR = (unsigned int)&led;        // Memory adress
		DMA1_Channel2->CNDTR = led_length*led_bits;                 // 8 data to transfer (8 ADC channels)
	}
	DMA1_Channel2->CCR |= DMA_CCR1_EN | DMA_CCR1_TCIE;        // Channel 1 Enable |  transfer complete interrupt
	
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	
}

void DMA1_Channel2_IRQHandler(void)
{
	if(DMA1->ISR & DMA_ISR_TCIF2)
	{
		DMA1->IFCR |= DMA_IFCR_CGIF2;
		_LED_dma_flag = 0;
		TIM2->SR &= ~ TIM_SR_UIF;
		TIM2->CR1 &= ~TIM_CR1_CEN;
	}
}

void _LED_init(void)
{
	uint8_t i = 0;
	
	led_gpio_init();
	led_timer_init();
	led_DMA_init(led_reset_state);
	led_bus_reset();	//0 on ouput for 24 bit long to reset the lines
	while (_LED_dma_flag){};
	led_DMA_init(led_on_state);
	led_refresh_timer_init();	
	led_brightness_coeff = 100;
	
	for(i = 0; i< led_length; i++)
	{
		led_blink_status[i] = led_blink_off;
	}
	animation_step = animation_off;
	led_current_animation = 100;
	led_animate_brightness = 5;
}

void _LED_off(void)
{
	uint8_t i = 0;
	
	for(i = 0; i<led_length*led_bits; i++)
	{
		led[i] = led_low;
	}
	
	led_timer_on();
}

void _LED_on(void)
{
	while (_LED_dma_flag){};
	_LED_set();
	led_timer_on();
	while (_LED_dma_flag){};
}

void _LED_set(void)
{
	uint8_t i	=	0;
	uint8_t j = 0;
	uint8_t red_s = 0;
	uint8_t blue_s = 0;
	uint8_t green_s = 0;
	uint32_t temp_state[led_length];	
	while (_LED_dma_flag){};
	for(j = 0; j<led_length; j ++)
	{
		if(led_blink_status[j] == led_blink_off || (LED_blinking_state == 1 && led_blink_status[j] == led_blink_on)  )
		{		
			red_s = reverse(((uint8_t)((led_state[j] >> color_red) & 0xFF)*led_brightness_coeff/100));
			blue_s = reverse(((uint8_t)((led_state[j] >> color_blue) & 0xFF))*led_brightness_coeff/100);
			green_s = reverse(((uint8_t)((led_state[j] >> color_green) & 0xFF)*led_brightness_coeff/100));
		}
		else if(LED_blinking_state == 0 && led_blink_status[j] == led_blink_on)  
		{
			red_s = 0;
			blue_s = 0;
			green_s = 0;
		}
			temp_state[j] = green_s<<16 | (red_s) <<8 | (blue_s);
				
				for(i = 0; i<led_bits; i++)
				{
					if(temp_state[j] & (1<<i))
					{
						led[24*(j)+i] = led_high;
					}
					else
					{
						led[24*(j)+i] = led_low;
					}
				}
	}
}

void LED_set_values(uint8_t led_nr_set, uint8_t blue_set_temp, uint8_t red_set_temp, uint8_t green_set_temp)
{
	uint32_t temp_state =	0;	
	
	temp_state = green_set_temp<<color_green | (red_set_temp) <<color_red | (blue_set_temp);
	
	led_state[(led_nr_set-1)] = 0 ;
	led_state[led_nr_set-1] = temp_state;
}

void _LED_set_color_list(uint8_t led_number, uint8_t index)
{
	_LED_set_color(led_number, led_colors[index][0], led_colors[index][1], led_colors[index][2]);
}

void _LED_set_color(uint8_t led_number, uint8_t blue, uint8_t red, uint8_t green)		// colors in %
{
	uint32_t temp_state =	0;
	uint8_t i = 0;
	uint8_t red_set = (uint8_t)(red *led_limit_max/100);
	uint8_t blue_set =  (uint8_t)(blue *led_limit_max/100);
	uint8_t green_set = (uint8_t)(green *led_limit_max/100);
	
	LED_set_values(led_number,blue_set,red_set,green_set);
}

uint8_t  _LED_change_color(uint8_t led_number, uint8_t color, uint8_t step, uint8_t direction)		// color taken from define  | step - in %
{
	uint32_t temp_state =	0;
	uint8_t i = 0;
	uint8_t return_value =0;
	uint8_t red_set = (uint8_t)((led_state[led_number - 1] >> color_red) & 0xFF);
	uint8_t blue_set = (uint8_t)((led_state[led_number - 1] >> color_blue) & 0xFF);
	uint8_t green_set = (uint8_t)((led_state[led_number - 1] >> color_green) & 0xFF);
	
	uint8_t color_temp = (uint8_t)((led_state[led_number - 1] >> color) & 0xFF);
	//uint8_t change =  (uint8_t)(255*step/100);
	uint8_t change = step;

	if(direction == led_increment)
	{
		if((color_temp+change)>=(led_limit_max)){color_temp = (led_limit_max); return_value = led_brightness_max;}
		else{color_temp += change; }
	}
	else if(direction == led_decrement)
	{
		if((color_temp-change)>= color_temp || (color_temp-change) == 0){color_temp = 0x00; return_value = led_brightness_min;}
		else{color_temp -= change; }
	}
	if(color == color_red){red_set = color_temp;}
	else if(color== color_blue){blue_set = color_temp;}
	else if(color== color_green){green_set = color_temp;}
	
	
	LED_set_values(led_number,blue_set,red_set,green_set);
	return return_value;
}
uint8_t  _LED_change_color_all( uint8_t color, uint8_t step, uint8_t direction)	
{
	uint32_t temp_state =	0;
	uint8_t return_value = 0;
	uint8_t led_number = 0;	
	uint8_t i = 0;
	uint32_t control_number = 0;
	
	for(led_number = 0;led_number < led_length;led_number++)
	{
		control_number+=_LED_change_color((led_number+1),color,step,direction);
	}
	if(control_number == led_brightness_min*led_length)
	{
		return_value = led_brightness_min;
	}
	else if(control_number == led_brightness_max*led_length)
	{
		return_value = led_brightness_max;
	}
	return return_value;
}


uint32_t _LED_change_brightness(uint8_t led_n, uint8_t step, uint8_t direction)
{
	uint32_t temp_state =	0;
	uint32_t return_value = 0;
	uint8_t i = 0;
	uint8_t red_set = 0;
	uint8_t blue_set = 0;
	uint8_t green_set = 0;
	uint8_t change = step;
	
	if(led_n == 0){led_n =1;}
	else if (led_n > (led_length)){led_n = led_length;}
	
	red_set = (uint8_t)((led_state[led_n - 1] >> color_red) & 0xFF);
	blue_set = (uint8_t)((led_state[led_n - 1] >> color_blue) & 0xFF);
	green_set = (uint8_t)((led_state[led_n - 1] >> color_green) & 0xFF);
	
	red_set += change;
	blue_set += change;
	green_set += change; 
	
//	if(red_set == led_limit_max && blue_set == led_limit_max  && green_set == led_limit_max)
//	{
//			return led_brightness_max;
//	}		
//	
//	if(direction == led_increment)
//	{
//		if((red_set+2*change)>=(led_limit_max)){red_set = (led_limit_max); return_value = led_brightness_max;}
//		else{red_set += change; }
//		
//		if((blue_set+2*change)>=(led_limit_max)){blue_set = (led_limit_max); return_value = led_brightness_max;}
//		else{blue_set += change; }		
//		
//		if((green_set+2*change)>=(led_limit_max)){green_set = (led_limit_max); return_value = led_brightness_max;}
//		else{green_set += change; }		
//	}
//	else if(direction == led_decrement)
//	{
//		if((red_set-2*change)<=0x00){red_set = 0; return_value = led_brightness_min;}
//		else{red_set -= change; }
//		
//		if((blue_set-2*change)<=0x00){blue_set = 0; return_value = led_brightness_min;}
//		else{blue_set -= change; }		
//		
//		if((green_set-2*change)<=0x00){green_set = 0; return_value = led_brightness_min;}
//		else{green_set -= change; }		
//	}


	LED_set_values(led_n,blue_set,red_set,green_set);
	return return_value;
}

uint32_t _LED_change_brightness_perc(uint8_t led_n, uint8_t perc)
{
	uint32_t temp_state =	0;
	uint32_t return_value = 0;
	uint8_t i = 0;
	uint8_t red = (uint8_t)((led_state[led_n - 1] >> color_red) & 0xFF);
	uint8_t blue = (uint8_t)((led_state[led_n - 1] >> color_blue) & 0xFF);
	uint8_t green = (uint8_t)((led_state[led_n - 1] >> color_green) & 0xFF);
	uint8_t red_set = 0;
	uint8_t blue_set = 0;
	uint8_t green_set = 0;
	
	red_set = (uint8_t)(red*perc/100);
	blue_set = (uint8_t)(blue*perc/100);
	green_set = (uint8_t)(red*perc/100);
	
	LED_set_values(led_n,blue_set,red_set,green_set);
		return return_value;
}

uint32_t _LED_change_brightness_all(uint8_t step, uint8_t direction)
{
	uint8_t led_number = 0;
	uint32_t return_value = 0;
	
	for(led_number = 1; led_number<=led_length; led_number++)
	{
		return_value += _LED_change_brightness(led_number, step, direction);
	}
	return_value = (uint32_t)(return_value/led_length);
	
	return return_value;
}

uint32_t _LED_change_brightness_all_perc(uint8_t perc)
{
	uint8_t led_number = 0;
	
	for(led_number = 1; led_number<=led_length; led_number++)
	{
		_LED_change_brightness_perc(led_number,perc);
	}
	
	return 0;	
}

void _LED_change_brightness_limit_list(uint8_t index)
{
	_LED_change_brightness_limit(led_brightness[index]);
}

void _LED_change_brightness_limit(uint8_t brightness)	// set limit for brightness in %
{
	led_brightness_coeff = brightness;
	_LED_change_brightness_all(0,led_increment);
	_LED_set();
}

void led_refresh_timer_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	
	_LED_refresh_flag = 0;
	
	TIM10->CR1 &= ~(TIM_CR1_CKD 	//clock division - none
						| TIM_CR1_ARPE			//TIMx_ARR register is not buffered
						| TIM_CR1_CMS		//Edge-aligned mode
						| TIM_CR1_DIR		// up counter
						| TIM_CR1_UDIS		//UEV enabled
						);	
	TIM10->CR1 |=	  TIM_CR1_OPM;		// one pulse mode
	
	
	TIM10->PSC = 31999;// prescaler value fCK_PSC / (PSC[15:0] + 1)		1ms tick
	TIM10->ARR = 0;	// 	auto-reload value 
	TIM10->CNT = 0;
	
	TIM10->DIER &= ~(TIM_DIER_UIE | TIM_DIER_UDE);		//disable all functions both - melody and alarm
	TIM10->CR2 &= ~TIM_CR2_CCDS;
	
	NVIC_EnableIRQ(TIM10_IRQn);		//uzywane podczas ostatniego sampla dzwieku!

	TIM10->DIER |= TIM_DIER_UIE;		// wlacz przerwanie

	TIM10->EGR |= TIM_EGR_UG;                               // Initialize all registers
	TIM10->SR &= ~ TIM_SR_UIF;				// czyszczenie flagi	
	
}

void _LED_blink_on(uint8_t led_number)		//start blinking chuj ci w dupe i tak tego nie czytasz :)
{
	if(led_blink_status[led_number-1] == led_blink_off)
	{
		led_blink_status[led_number-1] = led_blink_on;
		_LED_refresh_flag = 1;
	}
	else if(_LED_refresh_flag && led_blink_status[led_number-1] == led_blink_on)
	{
		_LED_refresh_flag = 0;
		_LED_on();
		_LED_refresh(400);
	}
}

void _LED_blink_off(uint8_t led_number)		// stop blinking
{
	led_blink_status[led_number-1] = led_blink_off;
	_LED_on();
	_LED_refresh_flag = 0;
}

void _LED_refresh(uint16_t delay_ms)
{
	if(!_LED_refresh_flag)
	{
		TIM10->ARR = delay_ms;
		_LED_refresh_flag = 0;	//clear flag
		TIM10->CR1 |= TIM_CR1_CEN;
	}
}

void _LED_animate_delay(uint16_t delay_ms)
{
	_LED_on();
	_LED_refresh_flag = 0;
	_LED_refresh(delay_ms);
}


void _LED_animate(void)
{
	static uint8_t led_number_static = 1;
	uint8_t i= 0;
	uint8_t temp_variable = 0;
	
	
		if(animation_step == animation_start)
		{
			for(i = 0; i< led_length; i++)
			{
				led_state[i] = 0;
			}
			_LED_on();
			_LED_refresh_flag = 1;
			led_number_static = 1;
			animation_step = animation_step_0;
			
		}
		
		if(_LED_refresh_flag)		//opóznienie sie skonczylo
		{
			if(led_current_animation == 0)
			{
				if(animation_step == animation_step_0)
				{
					temp_variable = _LED_change_brightness_all(1,led_increment);
					if(temp_variable == led_brightness_max )
					{
					
					}
					_LED_animate_delay(10);
				}
				else if(animation_step == animation_step_1)
				{
					if(_LED_change_brightness(led_number_static-1,10,led_decrement) == led_brightness_min )
					{
						if(led_number_static--<1)
						{
							animation_step = animation_step_2;
						}							
					}
					_LED_animate_delay(100);
				}
				else if(animation_step == animation_step_2)
				{
					if(_LED_change_brightness_all(1,led_increment) == led_brightness_max )
					{
						animation_step = animation_step_3;						
					}
					_LED_animate_delay(10);
				}
				else if(animation_step == animation_step_3)
				{
					if(_LED_change_brightness_all(1,led_decrement) == led_brightness_min )
					{
						animation_step = animation_step_4;						
					}
				}
				else if(animation_step == animation_step_4)
				{
				}
			}
				
			
			else if(led_current_animation == 1)
			{
				if(animation_step == animation_step_0)
				{
					_LED_set_color(1,100,100,100);
					_LED_set_color(2,30,30,30);
					_LED_set_color(3,15,15,15);
					_LED_set_color(4,3,3,3);
					_LED_set_color(5,0,0,0);
					_LED_animate_delay(10);
				}
				else if(animation_step == animation_step_1)
				{
				}
				else if(animation_step == animation_step_2)
				{
				}
				else if(animation_step == animation_step_3)
				{
				}
				else if(animation_step == animation_step_4)
				{
				}
			}
		}
		
		
		if(animation_step == animation_end)
		{
			for(i = 0; i< led_length; i++)
			{
				led_state[i] = led_state_temp[i];
			}
			_LED_on();			
			animation_step = animation_off;
		}
		
}

void _LED_animate_off(void)
{
	uint8_t i = 0 ;
	led_brightness_coeff = 	led_brightness_temp;
	for(i = 0; i< led_length; i++)
	{
		led_state[i] = led_state_temp[i];
	}
	_LED_refresh_flag = 1;
	_LED_on();			
	animation_step = animation_off;
}

void _LED_animate_change(uint8_t number)
{
	uint8_t i = 0;
	if(animation_step != animation_off)		//interrupt in current animation
	{
		led_current_animation = number;
		animation_step = animation_start;
	}
	else 
	{
		for(i = 0; i< led_length; i++)
		{
			led_state_temp[i] = led_state[i];
		}
		led_current_animation = number;
		animation_step = animation_start;
		led_brightness_temp = led_brightness_coeff;
		led_brightness_coeff = led_animate_brightness;
	}
}



void TIM10_IRQHandler(void)
{
	if(TIM10->SR & TIM_SR_UIF) // if UIF flag is set
	{
		TIM10->SR &= ~TIM_SR_UIF; // clear UIF flag
		_LED_refresh_flag = 1;		// set flag
		LED_blinking_state = !LED_blinking_state;
	}
}

void wait_(uint32_t time)
{
	uint32_t i = 0;
	for(i=0; i<time; i++){}
}

uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}