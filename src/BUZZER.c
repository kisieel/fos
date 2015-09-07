#include "BUZZER.h"
#include "stm32l1xx.h"

volatile struct beep_struct beep;
volatile struct music_library_struct music;
volatile struct alarm_struct alarm;

volatile uint8_t BUZZER_mode;

volatile uint8_t BUZZER_AlarmTones[buzzer_tones_qnt][2] = {
	{cT, eT},
	{dT, fT},
	{eT, gT},
	{fT, aT},
	{gT, bT},
	{aT, CT},
};

volatile uint8_t BUZZER_AlarmVols[buzzer_vols_qnt] = {
	0,
	49,
	50,
	52,
	53,
	255
};

volatile uint16_t BUZZER_AlarmTempos[buzzer_tempos_qnt] = {
	100,
	200,
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000
};

#define tempo_1s        1
#define tempo_2s        2
#define tempo_4s        4

#define note_32         31
#define note_16         62
#define note_8          125
#define note_4          250
#define note_2          500
#define note_1          1000

uint16_t music_0_tempo[] = {
	100,
	300,
	500,
	700,
	music_stop_sign
};

uint8_t music_0_tone[] = {
	cT,
	eT,
	dT,
	CT,
	0xFF
};

uint16_t music_1_tempo[] = {
	tempo_1s*note_4,
	tempo_1s*note_1,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_1,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_1,
	tempo_1s*note_4,
	
	tempo_1s*note_2,
	tempo_1s*note_2,
	tempo_1s*1250,
	tempo_1s*note_2,
	
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_1,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_1,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_4,
	tempo_1s*note_4,
	
	tempo_1s*note_1,
	tempo_1s*note_4,
	
	tempo_1s*note_2,
	tempo_1s*note_2,
	tempo_1s*1250,
	tempo_1s*note_2,
	
	music_stop_sign
};

uint8_t music_1_tone[] = {
	gT,
	0,
	
	gT,
	0,
	
	aT,
	0,
	
	gT,
	0,
	
	0,
	0,
	
	gT,
	0,
	
	aT,
	0,
	
	gT,
	0,
	
	0,
	0,
	
	gT,
	aT,
	CT,
	DT,
	
	0,
	
	gT,
	0,
	
	gT,
	0,
	
	aT,
	0,
	
	gT,
	0,
	
	0,
	0,
	
	gT,
	0,
	
	aT,
	0,
	
	gT,
	0,
	
	0,
	0,
	
	gT,
	aT,
	CT,
	DT,

	0xFF
};

uint16_t music_2_tempo[] = {
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	
	tempo_2s * note_2,
	tempo_2s * note_2,
	
	tempo_2s * note_16,
	tempo_2s * note_16,
	tempo_2s * note_16,
	
	tempo_2s * note_2,
	tempo_2s * note_4,
	
	tempo_2s * note_16,
	tempo_2s * note_16,
	tempo_2s * note_16,
	
	tempo_2s * note_2,
	tempo_2s * note_4,
	
	tempo_2s * note_16,
	tempo_2s * note_16,
	tempo_2s * note_16,
	
	tempo_2s * note_2,
	
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	tempo_2s * note_32,
	
	music_stop_sign
};

uint8_t music_2_tone[] = {
	CT,
	0,
	CT,
	0,
	CT,
	0,
	
	gT,
	cT,
	
	dT,
	eT,
	fT,
	
	xT,
	cT,
	
	dT,
	eT,
	fT,
	
	xT,
	cT,
	
	dT,
	eT,
	dT,
	
	fT,
	
	CT,
	0,
	CT,
	0,
	CT,
	0,
	
	0xFF
};

void BUZZER_GPIO_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                // Clock for GPIOB 

	
  GPIOB->MODER |= (GPIO_MODER_MODER13_1);             // Alternate function 
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_13 );              // Output push-pull
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13;      	//  10 MHz Medium speed

	GPIOB->AFR[1] |= (GPIO_AFRH_AFRH13 & (3 << 4*5));
}

void BUZZER_PWM_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; 
	
	TIM9->CR1 &= ~(TIM_CR1_CKD //clock division - none
						| TIM_CR1_ARPE			//TIMx_ARR register is not buffered
						| TIM_CR1_CMS		//Edge-aligned mode
						| TIM_CR1_DIR		// up counter
						| TIM_CR1_UDIS		//UEV enabled
						);
	TIM9->CR1 |=	 TIM_CR1_CMS_1  | TIM_CR1_URS;		// one pulse mode
	
	TIM9->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0 ;    // OC3, PWM mode 1 , upcounting
 
	TIM9->CCER |= TIM_CCER_CC1E  | TIM_CCER_CC1P;                          // OC3 enable
	
//	TIM9->DIER |= TIM_DIER_UIE;
	TIM9->PSC = 99;// prescaler value fCK_PSC / (PSC[15:0] + 1)		50ns tick
	TIM9->ARR = 0;	// 79	auto-reload value 
	TIM9->CCR1 = 0;	//	40 auto-reload value 
	TIM9->CNT = 0;

	TIM9->EGR |= TIM_EGR_UG;                               // Initialize all registers
	TIM9->SR &= ~ TIM_SR_UIF;				// czyszczenie flagi
}

void DAC_GPIO_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                // Clock for GPIOB 
	
  GPIOA->MODER |= (GPIO_MODER_MODER4);             // analog mode 
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;      	//  10 MHz Medium speed

}

void DAC_LOGIC_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR &= ~(DAC_CR_BOFF1 | DAC_CR_TEN1);	// triger disabled, output is in 1 clock cycle   | buffer enabled
	DAC->DHR8R1 = 0x00;	//vref * (dor/4095) = vout	
	DAC->CR |= DAC_CR_EN1;		// DAC enable
}

void BUZZER_reset_timer(void)
{
	BUZZER_mode = 0;	
	TIM9->CNT = 0;
	TIM9->ARR = 0;
	TIM9->CCR1 =  0;
	TIM3->ARR = 1;
	TIM3->CNT = 0;
	TIM9->CR1 &= ~TIM_CR1_CEN;	
	TIM3->CR1 &= ~TIM_CR1_CEN;
}


void _BUZZER_off(void)
{
	TIM3->SR &= ~ TIM_SR_UIF;				// czyszczenie flagi
	TIM9->SR &= ~ TIM_SR_UIF;				// czyszczenie flagi
	
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM9->CR1 &= ~TIM_CR1_CEN;
	
	TIM3->ARR = 1;
	TIM9->ARR = 0;
	
	vol_off;	
}

void _BUZZER_alarm_set_tone_list(uint8_t index)
{
	alarm.tone[0] = BUZZER_AlarmTones[index][0];
	alarm.tone[1] = BUZZER_AlarmTones[index][1];
	BUZZER_reset_timer();
}

void _BUZZER_alarm_set_vol_list(uint8_t index)
{
	alarm.volume = BUZZER_AlarmVols[index];
	DAC->DHR8R1 = alarm.volume;
}

void _BUZZER_alarm_set_tempo_list(uint8_t index)
{
	alarm.tempo = BUZZER_AlarmTempos[index];
	BUZZER_reset_timer();
}

void _BUZZER_alarm_start(void)
{
	BUZZER_mode = buzzer_mode_alarm;
	DAC->DHR8R1 = alarm.volume;
	TIM3->ARR = alarm.tempo;	
	if(!(TIM9->CR1 &= TIM_CR1_CEN)){
		TIM9->CR1 |= TIM_CR1_CEN;}
	if(!(TIM3->CR1 &= TIM_CR1_CEN)){
		TIM3->CR1 |= TIM_CR1_CEN;}
		
}

void _BUZZER_alarm_stop(void)
{
	BUZZER_reset_timer();
}

void BUZZER_music_init(void)
{
	music.samples[0].tempo = music_0_tempo;
	music.samples[0].tone = music_0_tone;
	
	music.samples[1].tempo = music_1_tempo;
	music.samples[1].tone = music_1_tone;
	
	music.samples[2].tempo = music_2_tempo;
	music.samples[2].tone = music_2_tone;
	
	music.volume = 0xFF;
}

void _BUZZER_play_music(uint8_t music_number)
{
	DAC->DHR8R1 = music.volume;
	if( BUZZER_mode == buzzer_mode_melody)
	{
		BUZZER_reset_timer();
		BUZZER_music_init();
		BUZZER_mode = buzzer_mode_melody;
	}
	else
	{
		BUZZER_mode = buzzer_mode_melody;
	}
	music.current_music_number = music_number;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void _BUZZER_stop_music(void)
{
		BUZZER_reset_timer();
		BUZZER_music_init();
}

void _BUZZER_beep_change(uint16_t length, uint16_t volume, uint16_t tone)
{
	beep.length = length;
	beep.tone = tone;
	beep.volume = volume;
}

void _BUZZER_single_beep(void)
{
	beep.beep_stop = 1;
	DAC->DHR8R1 = beep.volume;
	BUZZER_mode = buzzer_mode_beep;
	TIM3->CR1 |= TIM_CR1_CEN;		
}

void _BUZZER_init(void)
{
	DAC_GPIO_init();
	DAC_LOGIC_init();
	BUZZER_GPIO_init();
	BUZZER_PWM_init();
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	TIM3->CR1 &= ~(TIM_CR1_CKD 	//clock division - none
						| TIM_CR1_ARPE			//TIMx_ARR register is not buffered
						| TIM_CR1_CMS		//Edge-aligned mode
						| TIM_CR1_DIR		// up counter
						| TIM_CR1_UDIS		//UEV enabled
						);	
	TIM3->CR1 |=	  TIM_CR1_OPM;		// one pulse mode
	
	TIM3->PSC = 31999;// prescaler value fCK_PSC / (PSC[15:0] + 1)		1ms tick
	TIM3->ARR = 1;	// 	auto-reload value 
	TIM3->CNT = 0;
	
	alarm.position = 0 ;

	beep.tone = nominal_freuqency;
	beep.volume = 1;
	beep.length = 100;
	
	NVIC_EnableIRQ(TIM3_IRQn);
	TIM3->DIER |= TIM_DIER_UIE;		// wlacz przerwanie	
	TIM3->EGR |= TIM_EGR_UG;                               // Initialize all registers	
	TIM3->SR &= ~ TIM_SR_UIF;				// czyszczenie flagi
	BUZZER_reset_timer();
	BUZZER_music_init();
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
	{
		TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
		
		if(BUZZER_mode == buzzer_mode_melody)		//ostatni ton z melodii
		{
			
				if(*(music.samples[music.current_music_number].tempo) != music_stop_sign)
				{
					TIM9->CNT = 0;
					TIM9->ARR = *(music.samples[music.current_music_number].tone);
					TIM9->CCR1 = (uint16_t) *(music.samples[music.current_music_number].tone++) /2;
					TIM3->ARR = *(music.samples[music.current_music_number].tempo++);
					TIM3->CR1 |= TIM_CR1_CEN;
					TIM9->CR1 |= TIM_CR1_CEN;
				}
			
				else if(*(music.samples[music.current_music_number].tempo) == music_stop_sign)
				{
					BUZZER_music_init();
					BUZZER_reset_timer();
				}
		}
		else if(BUZZER_mode == buzzer_mode_alarm)
		{
			TIM9->CNT = 0;
			TIM9->ARR = alarm.tone[alarm.position];
			TIM9->CCR1 = (uint16_t) alarm.tone[alarm.position]/2;
			alarm.position = 1 - alarm.position;
			TIM3->CR1 |= TIM_CR1_CEN;		
		}
		else if(BUZZER_mode == buzzer_mode_beep)
		{
			if(beep.beep_stop)
			{
				TIM9->CNT = 0;
				TIM9->ARR = beep.tone;
				TIM9->CCR1 = (uint16_t) (beep.tone/2);
				TIM3->ARR = beep.length;
				TIM9->CR1 |= TIM_CR1_CEN;	
				TIM3->CR1 |= TIM_CR1_CEN;	
				beep.beep_stop = 0;
			}
			else if(!beep.beep_stop)
			{
				beep.beep_stop = 1;
				BUZZER_reset_timer();
			}				
		}
	}
}
