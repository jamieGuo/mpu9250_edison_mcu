/*
 * mympl.h
 *
 *  Created on: 2016/2/19
 *      Author: KuoJamie
 */

#ifndef MYMPL_H_
#define MYMPL_H_

#include "mcu_api.h"
#include "math.h"

int dmpGetEuler(float *data, long *q)	{

	debug_print(DBG_INFO, "euler!\n");
	long w = q[0];
	long x = q[1];
	long y = q[2];
	long z = q[3];

	debug_print(DBG_INFO, "test2.1\n");
	atan2( (double)w, (double)y );

	debug_print(DBG_INFO, "test3\n");
	data[0] = atan2( (2*x*y - 2*w*z)/1.f , (2*w*w + 2*x*x - 1)/1.f ); // psi

	debug_print(DBG_INFO, "test4\n");
	data[1] = -asin( 2*x*z + 2*w*y);	//theta

	debug_print(DBG_INFO, "test5\n");
	data[2] = atan2( 2*y*z - 2*w*x , 2*w*w + 2*z*z - 1 ); // phi

	debug_print(DBG_INFO, "test6\n");
	return 0;
}

#endif /* MYMPL_H_ */
