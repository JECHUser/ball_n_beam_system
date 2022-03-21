#include "include.h"
extern void UART0_ISR(void);

void ConfigUART0(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Internal Oscillator 16MHz
    UARTStdioConfig(0, 115200, 16000000);

	UARTIntRegister(UART0_BASE, &UART0_ISR);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	UARTEnable(UART0_BASE);
	IntEnable(INT_UART0);
}

//Bluetooth module interface
void ConfigUART1(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(1, 115200, 16000000);

    UARTIntEnable(UART1_BASE, UART_INT_RX); // enable RX interrupts
    //UARTIntRegister(UART1_BASE, &UARTIntHandler_UART1);
	UARTEnable(UART1_BASE);
    IntEnable(INT_UART1);
}

void UARTPut_int32(uint32_t UART_Base, int32_t num2send)
{

	int i;
	int32_t temp=num2send;
	for(i=4;i>0;i--)
	{
		UARTCharPutNonBlocking (UART_Base, (char)((temp&0xff000000)>>24));
		temp=temp<<8;
	}
}

void UARTPutn(uint32_t UART_Base, long Num)
{
	unsigned long temp = 1;
	long NumTemp;
	NumTemp = Num;
	if (Num == 0)
	{
		UARTCharPutNonBlocking(UART_Base, 48);
	}
	else
	{
		if (Num < 0)
		{
			UARTCharPutNonBlocking(UART_Base, '-');

			Num *= -1;
		}
		while (NumTemp)
		{
			NumTemp /= 10;
			temp *= 10;
		}
		temp /= 10;
		while (temp)
		{
			UARTCharPutNonBlocking(UART_Base,(Num / temp) % 10 + 48);

			temp /= 10;
		}
	}
}

void UARTPutFloat(uint32_t UART_Base, float fnum2send)
{
	uint8_t array2send[4];
	*((float *)array2send) = fnum2send;
	int i;
	for(i=0;i<4;i++)
	{
		UARTCharPutNonBlocking (UART_Base, array2send[i]);
	}
}

void UART0_ISR(void)
{
	uint32_t ui32Status;
	// Get the interrupt status.
	ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
	// Clear the asserted interrupts.
	ROM_UARTIntClear(UART0_BASE, ui32Status);
}


