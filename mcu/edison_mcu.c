/*
 * edison_mcu.c
 *
 *  Created on: 2015/11/22
 *      Author: KuoJamie
 */

#include <edison_mcu.h>

void __no_operation()	{};

inline int min(int a, int b)	{
	if( a > b )
		return b;
	return a;
}
int my_get_ms(unsigned long* count)	{
    if (!count)
        return 1;
    count[0] = time_ms();
    return 0;
};
