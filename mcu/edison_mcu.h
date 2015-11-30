#ifndef _EDISON_MCU_H_
#define _EDISON_MCU_H_

#include <mcu_api.h>
#include <mcu_errno.h>


void __no_operation();
inline int min(int a, int b);
int my_get_ms(unsigned long* count);


#define i2c_w(a, b, c, d)   i2c_write(a, b, d, c)
#define i2c_r(a, b, c, d)   i2c_read(a, b, d, c)
///* delay_ms is a function already defined in ASF. */
#define delay_ms(ms)    	mcu_delay(ms*1000)
#define get_ms  			my_get_ms
static inline int reg_int_cb(struct int_param_s *int_param)
{
//    sensor_board_irq_connect(int_param->pin, int_param->cb, int_param->arg);
    return 0;
}
#define log_i(...)       	debug_print(DBG_INFO, __VA_ARGS__);
#define log_e(...)       	debug_print(DBG_ERROR, __VA_ARGS__);
///* UC3 is a 32-bit processor, so abs and labs are equivalent. */
#define labs        abs
#define fabs(x)     (((x)>0)?(x):-(x))

#endif
