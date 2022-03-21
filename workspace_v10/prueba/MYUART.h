#ifndef UART_H_
#define UART_H_

//#define UART_GUI_BLUETOOTH_BASE		UART1_BASE
void ConfigUART0(void);
void ConfigUART1(void);
void UARTPut_int32(uint32_t UART_Base, int32_t num2send);
void UARTPutn(uint32_t UART_Base, long Num);
void UARTPutFloat(uint32_t UART_Base, float fnum2send);
void UART0_ISR(void);

#endif /* UART_H_ */
