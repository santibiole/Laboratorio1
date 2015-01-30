#include <stdint.h>
#include <math.h>
#include "bsp/bsp.h"

/**
 * @brief Delay por software
 *
 * @param nCount Numero de ciclos del delay
 */
void Delay(volatile uint32_t nCount);

/**
 * @brief Se encarga de prender un led y apagarlo luego de un tiempo
 *
 * @param led    Numero de led a pulsar
 * @param tiempo Numero de ciclos del delay entre prendido y apagado
 */
void pulsoLed(uint8_t led, uint32_t tiempo);

/**
 * @brief Aplicacion principal
 */
int main(void) {
	bsp_init();
	float brillo = 0;
	float intensidad_rojo = 0;
	float intensidad_verde = 0;
	float intensidad_azul = 0;

	led_set_bright(0, brillo);
	led_set_bright(1, brillo);
	led_set_bright(2, brillo);

	while (1) {

		bsp_delay_ms(10);

		brillo = adc()/100;

		if (brillo >= 0 && brillo < 0.25){
			intensidad_rojo = 100;
			intensidad_verde = brillo*100*4;
			intensidad_azul = 0;
		}
		if (brillo >= 0.25 && brillo < 0.5){
			intensidad_rojo = (100-((brillo-0.25)*100*4));
			intensidad_verde = 100;
			intensidad_azul = ((brillo-0.25)*100*4);
		}
		if (brillo >= 0.5 && brillo < 0.75){
			intensidad_rojo = 0;
			intensidad_verde = (100-((brillo-0.5)*100*4));
			intensidad_azul = 100;
		}
		if (brillo >= 0.75 && brillo <= 1){
			intensidad_rojo = ((brillo-0.75)*100*4);
			intensidad_verde = 0;
			intensidad_azul = (100-((brillo-0.75)*100*4));
		}

		led_set_bright(0,intensidad_rojo);
		led_set_bright(1,intensidad_verde);
		led_set_bright(2,intensidad_azul);

	}
}

void pulsoLed(uint8_t led, uint32_t tiempo){
	led_on(led);
	Delay(tiempo);
	led_off(led);
}

void Delay(volatile uint32_t nCount) {
	while (nCount--) {
	}
}

void APP_ISR_sw (void){

}

void APP_ISR_1ms (void){
	static uint16_t count_1s = 1000; // static: es una variable que se declara una sola vez, se fija en 0, y luego cada vez que entramos en la función, count conserva el valor anterior.
	count_1s--;
	if (!count_1s) {
		count_1s = 1000;
	}
}
