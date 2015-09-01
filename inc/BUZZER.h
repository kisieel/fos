#include "stm32l1xx.h"

#define vol_max 	DAC->DHR8R1 = 0xFF;	//vref * (dor/4095) = vout	
#define vol_mid 	DAC->DHR8R1 = 47;	//vref * (dor/4095) = vout
#define vol_min 	DAC->DHR8R1 = 44;	//vref * (dor/4095) = vout
#define vol_off		DAC->DHR8R1 = 0x00;

#define nominal_freuqency 79

#define cT  nominal_freuqency - 15
#define dT	nominal_freuqency - 10
#define eT	nominal_freuqency - 5
#define fT	nominal_freuqency
#define gT	nominal_freuqency + 5
#define aT	nominal_freuqency + 10
#define bT	nominal_freuqency + 15
#define CT	nominal_freuqency + 20

#define music_length 10
#define max_music_samples 10


#define alarm_high nominal_freuqency + 10
#define alarm_low	 nominal_freuqency - 10
#define alarm_period_define		400

#define buzzer_mode_alarm			10
#define buzzer_mode_melody		20
#define buzzer_mode_beep			30

//uint8_t temp_tone[10] = {CT,dT,CT,dT,CT,dT,CT,dT,CT,dT};
//uint16_t temp_tempo[10] = {200,200,200,200,500,500,1000,500,500,500};

struct music_struct
{
	uint16_t tempo[music_length];
	uint16_t tone[music_length];
	uint8_t position;
};

struct music_library_struct
{
	struct music_struct samples[max_music_samples];
	uint8_t current_music_number;
};

struct alarm_struct
{
	uint16_t tempo;
	uint16_t tone[2];
	uint8_t position;
};

struct beep_struct
{
	uint16_t tone;
	uint16_t volume;
	uint16_t length;
	uint8_t	beep_stop;
};

extern volatile struct music_library_struct music;
extern volatile struct alarm_struct alarm;
extern volatile struct beep_struct beep;

extern volatile uint8_t BUZZER_mode;

void BUZZER_GPIO_init(void);
void BUZZER_PWM_init(void);
void DAC_GPIO_init(void);
void DAC_LOGIC_init(void);
void BUZZER_DMA_MELODY_init(void);
void BUZZER_TIMER_change_config_last(void);
void BUZZER_reset_timer(void);

void _BUZZER_off(void);
void _BUZZER_play_music(uint8_t music_number);
void _BUZZER_alarm_stop(void);
void _BUZZER_alarm_start(void);
void _BUZZER_single_beep(void);
void _BUZZER_init(void);