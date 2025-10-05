#include <Ticker.h>

volatile int tick;

Ticker ticker_handle;

void tick_proc(int stat)
{
	tick++;
}

void setup()
{
	Serial.begin(115200);
	delay(100);
	ticker_handle.attach_ms(1, tick_proc, 0);
}

void loop()
{
	Serial.printf("sec = %4d(%d) \n",tick/1000,tick);
}

// ---------------------------------------------------------------
