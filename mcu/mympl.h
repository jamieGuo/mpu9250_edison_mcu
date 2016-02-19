/*
 * mympl.h
 *
 *  Created on: 2016/2/19
 *      Author: KuoJamie
 */

#ifndef MYMPL_H_
#define MYMPL_H_

#include "math.h"

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}


int dmpGetEuler(double *data, long *q)	{


	double a = 0.1;
	pow(2.0, 3.0);
	atan2(a,a);

	double w = q[0];
	double x = q[1];
	double y = q[2];
	double z = q[3];

	data[0] = atan2( (double)(2*x*y - 2*w*z) , (double)(2*w*w + 2*x*x - 1) ); // psi
	data[1] = -asin( 2*x*z + 2*w*y);	//theta
	data[2] = atan2( 2*y*z - 2*w*x , 2*w*w + 2*z*z - 1 ); // phi

	return 0;
}

#endif /* MYMPL_H_ */
