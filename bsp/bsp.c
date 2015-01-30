#include <stdint.h>
#include "stm32f4xx.h"			// Header del micro
#include "stm32f4xx_adc.h"		// Módulo analógico/digital
#include "stm32f4xx_exti.h"		// Controlador interrupciones externas
#include "stm32f4xx_gpio.h"		// Perifericos de E/S
#include "stm32f4xx_rcc.h"		// Para configurar el (Reset and clock controller)
#include "stm32f4xx_syscfg.h"	// configuraciones Generales
#include "stm32f4xx_tim.h"		// Modulos Timers
#include "misc.h"				// Vectores de interrupciones (NVIC)
#include "bsp.h"

#define RGB_R GPIO_Pin_0
#define RGB_G GPIO_Pin_1
#define RGB_B GPIO_Pin_4

#define BOTON GPIO_Pin_0

#define RESOLUCION_ADC	4095
#define PORCENTAJE 100

/* Puertos de los leds disponibles */
GPIO_TypeDef* leds_port[] = {GPIOB, GPIOB, GPIOB};
/* Leds disponibles */
const uint16_t leds[] = {RGB_R, RGB_G, RGB_B};

/* Leds disponibles PWM*/
uint32_t* const leds_pwm[] = {&TIM3->CCR3, &TIM3->CCR4, &TIM3->CCR1}; // Contiene punteros a variables del tipo unit32_t a distintas direcciones de memoria.

/*
	Prototipo de una función externa, es decir, una función que va a estar implementada en algún otro
	lugar de nuestro proyecto. El linker es el que se va a encargar de ubicar donde está implementada.
 */
extern void APP_ISR_sw (void);
extern void APP_ISR_1ms (void);

volatile uint16_t bsp_count_ms = 0; // Defino como volatile para que el compilador no interprete el while(bsp_count_ms) como un bucle infinito.

void led_on(uint8_t led) {
	GPIO_SetBits(leds_port[led], leds[led]);
}

void led_off(uint8_t led) {
	GPIO_ResetBits(leds_port[led], leds[led]);
}

void led_toggle(uint8_t led) {
	GPIO_ToggleBits(leds_port[led], leds[led]);
}

uint8_t sw_getState(void) {
	return GPIO_ReadInputDataBit(GPIOA, BOTON);
}

void led_set_bright (uint8_t led, uint8_t value){
	*leds_pwm[led] = 10000 * value / 100; // Porcentaje.
}

void bsp_delay_ms(uint16_t x){
	bsp_count_ms = x;
	while (bsp_count_ms);
}

float adc (void){
	return (read_adc()*PORCENTAJE/RESOLUCION_ADC);
}

/**
 * @brief Interrupcion llamada cuando se preciona el pulsador
 */
void EXTI0_IRQHandler(void) {

	if (EXTI_GetITStatus(EXTI_Line0) != RESET) { //Verificamos si es la del pin configurado.
		EXTI_ClearFlag(EXTI_Line0); // Limpiamos la Interrupcion.
		// Rutina:
		APP_ISR_sw();
		//GPIO_ToggleBits(leds_port[1], leds[1]);
	}

}

/**
 * @brief Interrupcion llamada al pasar 1ms
 */
void TIM2_IRQHandler(void) {

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		// Rutina:
		APP_ISR_1ms();

		if (bsp_count_ms) { //Pregunto si bsp_count_ms es distinto de 0.
			bsp_count_ms--;
		}
	}
}

void bsp_init_adc();
void bsp_led_init();
void bsp_pwm_init();
void bsp_sw_init();
void bsp_timer_init();

void bsp_init() {
	bsp_adc_init();
	//bsp_led_init();
	bsp_pwm_init();
	bsp_sw_init();
	bsp_timer_init();
}

/**
 * @brief Inicializa Leds
 */
void bsp_led_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // (Push/Pull)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief Inicializa SW
 */
void bsp_sw_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configuro interrupcion

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0); // Pin 0 del puerto A, lo hace interrupción.

	/* Configuro EXTI Line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0; // Interrupción en Línea 0.
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // Modo "Interrupción".
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Interrupción por flanco ascendente.
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Habilito la EXTI Line Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; // Qué el canal sea el de la interrupción 0.
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // Prioridad.
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; // Canal habilitado, habilito la interrupción.
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Habilito la interrupción.
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Inicializa TIM2
 */
void bsp_timer_init(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Habilito la interrupcion global del  TIM2 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM2 habilitado */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Configuracion de la base de tiempo */
	TIM_TimeBaseStruct.TIM_Period = 1000; // 1 MHz bajado a 1 KHz (1 ms). ¿Cómo? Cuento 1us, aumento el contador, y cuando llego a mil tengo 1ms, es decir 1KHz.
	TIM_TimeBaseStruct.TIM_Prescaler = (2 * 8000000 / 1000000) - 1; // 8 MHz bajado a 1 MHz - Pre Escalador.
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Divisor.
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up; // Como queremos que cuente.
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct); // Inicializamos timer.
	/* TIM habilitado */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // Inicializamos la interrupción.
	/* TIM2 contador habilitado */
	TIM_Cmd(TIM2, ENABLE);
}

void bsp_pwm_init(void) {
	TIM_TimeBaseInitTypeDef TIM_config;
	GPIO_InitTypeDef GPIO_config;
	TIM_OCInitTypeDef TIM_OC_config;

	/* Habilito el clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Configuro leds como Segunda Funcion */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_config.GPIO_Mode = GPIO_Mode_AF;
	GPIO_config.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_config.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_config.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_config.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOB, &GPIO_config);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

	TIM_config.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_config.TIM_ClockDivision = 0;
	TIM_config.TIM_Period = 10000;
	TIM_config.TIM_Prescaler = 16 - 1;
	TIM_TimeBaseInit(TIM3, &TIM_config);

	TIM_OC_config.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 0;
	TIM_OC_config.TIM_OCPolarity = TIM_OCPolarity_Low;

	// CH1 del pwm
	TIM_OC1Init(TIM3, &TIM_OC_config);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//CH3 del pwm
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 0;

	TIM_OC3Init(TIM3, &TIM_OC_config);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//CH4 del pwm
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 0;

	TIM_OC4Init(TIM3, &TIM_OC_config);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief Inicializa ADC
 */
void bsp_adc_init(void) {
	// Config structs
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_InitTypeDef ADC1_InitStruct;
	// Enable the clock for ADC and the ADC GPIOs

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// Configure these ADC pins in analog mode using GPIO_Init();
	GPIO_StructInit(&GPIO_InitStruct); // Reset gpio init structure
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; // Obvezno AIN !!!
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Common ADC init sets the prescaler
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStruct);
	/* ADC1 Configuration */
	ADC_StructInit(&ADC1_InitStruct);
	ADC1_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADC1, &ADC1_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	/* Now do the setup */
	ADC_Init(ADC1, &ADC1_InitStruct);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
}

uint16_t read_adc() {
	ADC_RegularChannelConfig(ADC1, 12, 1, ADC_SampleTime_15Cycles);
	ADC_SoftwareStartConv(ADC1);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET);
	return ADC_GetConversionValue(ADC1);
}
