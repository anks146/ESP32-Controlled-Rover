#include "UART.h"
#include "main.h"
void Tx_data(char c)
{
	while((UART4->ISR&(0X40))!=0X40);
	UART4->TDR=c;
	
	return;
}
//----------------------------------------------------------------
void Sendserial(char *s)
{
	int k=0;
	while(s[k]!='\0')
	{
		Tx_data(s[k]);
		k++;
	}
	return;
}
//----------------------------------------------------------------
