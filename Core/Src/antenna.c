#include "adc_data.h"
#include "filter.h"

uint8_t antenna_sensitivity = 0;
uint8_t antenna_gain = 0;

extern I2C_HandleTypeDef hi2c2;

/*		TMUX1121DGKR аналоговый переключатель 5V
	SEL1 = ON -- коммутирует S1(Source pin 1) и D1(Drain pin 1)
	SEL1 и SEL2 подключают параллельно сопротивление к R29 = 10MOm, S1 подключен к R26 = 560kOm, S2 к R27 = 10MOm

		SEL1  SEL2  R
		 0		0			10 	MOm 	max чувствительность антенны
		 0		1			5 	MOm
		 1		0			530 kOm
		 1		1			504 kOm		min чувствительность антенны
*/

#define MCP_ADDR (0x2F << 1)//0101111
/*		MCP4017T-104E/LT
			0x7F  		Full Scale	(A wiper setting of 7Fh connects theTerminal W (wiper) to Terminal A (Full Scale))
	0x7E - 0x40
			0x3F 			Mid Scale
	0x3E - 0x01
			0x00  		Zero Scale	(A wiper setting of 00h connects the Terminal W (wiper)to Terminal B (Zero Scale))

	N = 0 to 127 (decimal)
	R_AB = 100 kOm
	R_S = R_AB / 127
	R_S  = 787,402 Om 	(Step resistance (RS) is the resistance from one tapsetting to the next)
	здесь должно быть что-то ещё, но я не стал разбираться дальше. А зачем...
	При N = 0 сопротивление стремится к нулю
*/
void antenna_tuning(void) {

	//усиление, от 0(max усиление) до 127(min усиление)
	static uint8_t command[1] = {0};

	if(antenna_gain != command[0]) {
		command[0] = antenna_gain;
		HAL_I2C_Master_Transmit(&hi2c2, MCP_ADDR, command, 1, 10);
	}

	//чувствительность
	if(antenna_sensitivity & 0x01)
		SEL2_GPIO_Port->BSRR |= SEL2_Pin;
	else
		SEL2_GPIO_Port->BSRR |= (SEL2_Pin << 16);

	if(antenna_sensitivity & 0x02)
		SEL1_GPIO_Port->BSRR |= SEL1_Pin;
	else
		SEL1_GPIO_Port->BSRR |= (SEL1_Pin << 16);
}
