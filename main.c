#include "mcu_api.h"
#include "mcu_errno.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"
//#include "mylib.h"

int myconfi();
int mpu9250_read_fifo(int t);
void readRaw();
//
void mcu_main()	{

	mcu_sleep(100);

	while(mpu_init( (struct int_param_s *)0 ))	{
		debug_print(DBG_ERROR, "mpu init failed\n");
	}
	debug_print(DBG_INFO, "mpu init success\n");

	while( myconfi() )	{
		debug_print(DBG_ERROR, "configurate() failed\n");
	}
	debug_print(DBG_INFO, "configurate() success\n");

//	readRaw();
	while( mpu9250_read_fifo(30) )	{
		debug_print(DBG_DEBUG, "mpu9250_read_fifo() failed\n");
	}

	debug_print(DBG_DEBUG, "program finish() \n");
	//	return 0;
}

int myconfi()	{

	debug_print(DBG_ERROR, "myconfig() now...\n");


	if( mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) )	{
		debug_print(DBG_ERROR, "mpu_set_sensors() failed\n");
		return -1;
	}
	if (mpu_set_gyro_fsr(250))	{
		debug_print(DBG_INFO, "gyro fsr failded(250)\n");
	    return -1;
	}
	if( mpu_configure_fifo( INV_XYZ_GYRO | INV_XYZ_ACCEL ) )	{
		debug_print(DBG_ERROR, "mpu_configure_fifo() failed\n");
		return -1;
	}
	if( mpu_set_sample_rate( 50 ) )	{
		debug_print(DBG_ERROR, "mpu_set_sample_rate(50) failed\n");
		return -1;
	}
	if(mpu_set_compass_sample_rate(50)){
		debug_print(DBG_ERROR, "mpu_set_compass_sample_rate(50)\n");
		return -1;
	}

	if( dmp_load_motion_driver_firmware() )	{
		debug_print(DBG_ERROR, "dmp_load_motion_driver_firmware() failed\n");
		return -1;
	}
	debug_print(DBG_INFO, "dmp_load_motion_driver_firmware suc\n");

	signed char gyro_orientation[9] = { 1, 0, 0,
										0, -1, 0,
										0, 0, -1 };
	if( dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)) )	{
		debug_print(DBG_ERROR, "dmp_set_orientation failed\n ");
		return -1;
	}

	if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) )	{
		debug_print(DBG_INFO, "dmp_enable() failed\n");
		return -1;
	}

	if( dmp_set_fifo_rate(200) )	{
		debug_print(DBG_INFO, "dmp_set_fifo_rate(50) failed\n");
		return -1;
	}

	if( mpu_set_dmp_state(1)) 	{
		debug_print(DBG_INFO, "dmp(1) failed\n");
		return -1;
	}
	return 0;
};

int mpu9250_read_fifo(int t)	{

	debug_print(DBG_INFO, "Read DMP!\n");

	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors = {INV_XYZ_GYRO | INV_XYZ_ACCEL};
	unsigned char more;

//	short *gyro, short *accel, long *quat,
//	    unsigned long *timestamp, short *sensors, unsigned char *more)

	while(1){
		if( dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) < 0 )	{
			debug_print(DBG_INFO, "dmp_read_fifo() failed, more=%d\n", more );
			return -1;
		}
		debug_print(DBG_INFO, "[DMP][%d] ax:%d, ay:%d, az:%d) \t gx:%d, gy:%d, gz:%d \t q0: %d, q1: %d, q2: %d, q3: %d \n",
				timestamp, accel[0], accel[1], accel[2] , gyro[0], gyro[1], gyro[2], quat[0], quat[1], quat[2], quat[3] );

		mcu_sleep(t);
	}
	return 0;
}

void readRaw()	{
	short accel[3];
	short gyro[3];
	unsigned long ta, tg;
	while(1)	{
		mpu_get_accel_reg(accel, &ta);
		mpu_get_gyro_reg(gyro, &tg);
		debug_print(DBG_INFO, "[%d] a:%d, %d, %d \n [%d] g:%d, %d, %d\n", ta , accel[2], accel[1], accel[0]
		                                                                , tg, gyro[2], gyro[1], gyro[0] );
		mcu_sleep(40);
	}
}
