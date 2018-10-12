#include "mbed.h"
#include "DAC_CS43L22.h"
#include "SineTable.h"
#include "I2S.h"

#if defined(TARGET_DISCO_F407VG)
DigitalOut led1(LED3);

I2C             i2c(PB_9, PB_6);
DAC_CS43L22     dac(i2c, PD_4);
I2S             i2s(PC_12, PC_10, PA_4, NC, PC_7);
#else
#   error "target not supported"
#endif

void i2s_callback(int narg)
 {
    //printf("i2s callback, arg: 0x%x\n", narg);
//     if (!(narg & (I2S_EVENT_TX_COMPLETE | I2S_EVENT_TX_HALF_COMPLETE)))
//         error("Unexpected transmission event.\r\n");
//     else if ((narg & I2S_EVENT_TX_COMPLETE) && !_loop)
//         dac.stop();
 }

// main() runs in its own thread in the OS
int main() {
	printf("Hello world from DISCO_F407VG\n");
	
	i2c.frequency(100e3);

	dac.init(OUTPUT_DEVICE_HEADPHONE, 80, 0);
	int id = dac.readID();
	printf("DAC id: 0x%x\n", id);
	dac.play();

	i2s.mode(MASTER_TX, true);
	int err = i2s.transfer<uint16_t>(SINE_TABLE, sizeof(SINE_TABLE),    // tx buffer, tx size
	                                 NULL, 0,                           // rx buffer, rx size
	                                 &i2s_callback, I2S_EVENT_ALL);     // callback, callback events
	printf("i2s transfer error: %d\n", err);

	while (true)
    {
        led1 = !led1;

        i2s.i2s_bh_queue.dispatch(100);
    }

}

