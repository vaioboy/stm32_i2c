#include "main.h"

uint8_t i2c_write_data[] = "1234567890abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz";
uint8_t i2c_rx_data[255];
uint8_t test_num = 10;
uint8_t test_addr = 10;

int main(void)
{
	mini_config();
	
	LED_D4_OFF;
	LED_D5_OFF;
	
	printf("Hello PC!\r\n");

	while (1)
	{
	}
}

void mini_config()
{
	mini_led_init();
	mini_pb_config();
	mini_uart_config();
	mini_i2c_init();
}
