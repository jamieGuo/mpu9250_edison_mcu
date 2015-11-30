/*
 * myinclude.c
 *
 *  Created on: 2015/11/23
 *      Author: KuoJamie
 */

#include "mylib.h"

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;		// 000
    else if (row[0] < 0)
        b = 4;		// 100
    else if (row[1] > 0)
        b = 1;		// 001
    else if (row[1] < 0)
        b = 5;		// 101
    else if (row[2] > 0)
        b = 2;		// 010
    else if (row[2] < 0)
        b = 6;		//110
    else
        b = 7;		// error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)	{

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
