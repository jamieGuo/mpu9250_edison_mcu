#include "mcu_api.h"
#include "mcu_errno.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"	// for oriented
#include "mympl.h"

int myconfi();
void print_int_status();
int int_read_fifo();


void mcu_main()	{

	/* initial */
	while(mpu_init( (struct int_param_s *)0 ))	{
		debug_print(DBG_ERROR, "mpu init failed\n");
	}
	debug_print(DBG_INFO, "mpu init success\n");

	/* configure */
	while( myconfi() )	{
		debug_print(DBG_ERROR, "configurate() failed\n");
	}
	debug_print(DBG_INFO, "configurate() success\n");

	/* run */
	gpio_setup(48, 0);  /* set GPIO 48 as input*/
	gpio_register_interrupt(48, 1, int_read_fifo );
	while(1)	{
		//debug_print(DBG_INFO, "in loop\n");
		//mcu_sleep(10);
	}

//	readRaw(30);
//	while( mpu9250_read_fifo(30) )	{
//		debug_print(DBG_DEBUG, "mpu9250_read_fifo() failed\n");
//	}
//
	debug_print(DBG_DEBUG, "program finish() \n");
}

int myconfi()	{

//	inv_enable_quaternion();

	debug_print(DBG_ERROR, "myconfig() now...\n");

	if( mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) )	{
		debug_print(DBG_ERROR, "mpu_set_sensors() failed\n");
		return -1;
	}
	if (mpu_set_gyro_fsr(250))	{
		debug_print(DBG_INFO, "gyro fsr failded(250)\n");
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
	/* dmp setting */
	if( dmp_load_motion_driver_firmware() )	{
		debug_print(DBG_ERROR, "dmp_load_motion_driver_firmware() failed\n");
		return -1;
	}
	debug_print(DBG_INFO, "dmp_load_motion_driver_firmware suc\n");

	// original
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

	if( mpu_set_dmp_state(1) ) 	{
		debug_print(DBG_INFO, "mpu_set_dmp_state(1) failed\n");
		return -1;
	}
	if( dmp_set_interrupt_mode(DMP_INT_CONTINUOUS) )	{
		debug_print(DBG_INFO, "dmp_set_interrupt_mode(1) failed\n");
		return -1;
	}
	return 0;
};


void print_int_status()	{
	short status=0;
	mpu_get_int_status( &status );
	debug_print(DBG_INFO, "int_status: %d\n", status);
}
int int_read_fifo()	{

	int length = 0;
	char buf[256];

	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors = {INV_XYZ_GYRO | INV_XYZ_ACCEL};
	unsigned char more;

	if( dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) < 0 )	{
		debug_print(DBG_INFO, "dmp_read_fifo() failed, more=%d\n", more );
		return -1;
	}

//	debug_print(DBG_INFO, "test\n");
//	float euler[3];
//	dmpGetEuler( euler, quat);
//	debug_print(DBG_INFO, "[%dms] accel: %f, %f, %f , gyro: %f, %f, %f , euler: %f %f %f \n",
//							accel[0]/16384, accel[1]/16384, accel[2]/16384,
//							gyro[0]/131, gyro[1]/131, gyro[2]/131,
//							euler[0], euler[1], euler[2] );
	// ttymcu0
	length = mcu_snprintf( buf, 255, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			timestamp, accel[0], accel[1], accel[2] , gyro[0], gyro[1], gyro[2], quat[0], quat[1], quat[2], quat[3] );

	host_send( (unsigned char*)buf, length);

	// ttymcu1
//	debug_print(DBG_INFO, "[DMP][%d]ax:%d,ay:%d,az:%d)\tgx:%d,gy:%d,gz:%d\tq0:%d,q1:%d,q2:%d,q3:%d\n",
//			timestamp, accel[0], accel[1], accel[2] , gyro[0], gyro[1], gyro[2], quat[0], quat[1], quat[2], quat[3] );

	return 0;
}
