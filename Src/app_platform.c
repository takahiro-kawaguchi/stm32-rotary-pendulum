#include "main.h"
#include "edukit_system.h"
#include <stdio.h>
#include <string.h>

volatile uint16_t gLastError;

uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos,
		uint16_t BufMaxLen, T_Serial_Msg *Msg) {
	uint16_t NumNewByte = 0;
	uint8_t Data;
	uint16_t MsgIdx;
	uint16_t BuffIdx;

	if (LastPos >= StartPos) {
		NumNewByte = LastPos - StartPos;
	} else {
		NumNewByte = BufMaxLen + LastPos - StartPos;
	}
	BuffIdx = StartPos;

	for (MsgIdx = 0; MsgIdx < NumNewByte; MsgIdx++) {
		Data = CircularBuff[BuffIdx];
		BuffIdx++;
		if (BuffIdx >= BufMaxLen) {
			BuffIdx = 0;
		}

		if (Data == SERIAL_MSG_EOF) {
			Msg->Len = MsgIdx;
			/* Msg->Data is a single reused global buffer (Src/main.c) — without
			 * a null terminator here, a short message following a longer one
			 * (e.g. "q" right after "u -692.3") leaves stale trailing bytes
			 * that break any strcpy/strcmp treating Msg->Data as a C string
			 * (see ui_process_runtime_input()'s "q" check, Src/ui.c). */
			if (MsgIdx < SERIAL_MSG_MAXLEN) {
				Msg->Data[MsgIdx] = '\0';
			}
			return MsgIdx + 1;
		} else {
			Msg->Data[MsgIdx] = Data;
		}
	}
	return 0;
}

void MX_TIM3_Init(void) {
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(0);
	}
}

void MX_USART2_UART_Init(void) {
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 230400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler(0);
	}

	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
		Error_Handler(0);
	}
	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
}

/* L6474 FLAG-pin ISR (attached in app_bootstrap.c). Every branch below was
 * an empty stub -- any fault the driver reported here was silently
 * discarded. In particular L6474_STATUS_HIZ means the driver has put its
 * output bridge into high-impedance (motor electrically disconnected from
 * drive) while the rest of the firmware keeps computing/echoing u as if
 * nothing happened. That's a strong candidate for "commands are accepted
 * and reported back correctly but the rotor never physically turns" --
 * logging here (2026-07-19) so the next occurrence shows exactly which
 * condition tripped instead of leaving it a total mystery. */
void MyFlagInterruptHandler(void) {
	uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

	if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ) {
		sprintf(uart_tx_buf, "L6474 FLAG: HIZ (output disabled) status=0x%04X\r\n", statusRegister);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR) {
	} else {
	}

	if ((statusRegister & L6474_STATUS_NOTPERF_CMD)
			== L6474_STATUS_NOTPERF_CMD) {
		sprintf(uart_tx_buf, "L6474 FLAG: command not performed status=0x%04X\r\n", statusRegister);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD) {
		sprintf(uart_tx_buf, "L6474 FLAG: wrong command status=0x%04X\r\n", statusRegister);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	if ((statusRegister & L6474_STATUS_UVLO) == 0) {
		sprintf(uart_tx_buf, "L6474 FLAG: under-voltage lockout status=0x%04X\r\n", statusRegister);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	if ((statusRegister & L6474_STATUS_TH_WRN) == 0) {
		sprintf(uart_tx_buf, "L6474 FLAG: thermal warning status=0x%04X\r\n", statusRegister);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	if ((statusRegister & L6474_STATUS_TH_SD) == 0) {
		sprintf(uart_tx_buf, "L6474 FLAG: thermal shutdown status=0x%04X\r\n", statusRegister);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	if ((statusRegister & L6474_STATUS_OCD) == 0) {
		sprintf(uart_tx_buf, "L6474 FLAG: over-current detected status=0x%04X\r\n", statusRegister);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}
}

void Error_Handler(uint16_t error) {
	gLastError = error;

	while (1) {
	}
}
