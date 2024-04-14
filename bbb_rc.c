/**
 * @file bbb_rc.c
 *
 * 
 */

#include "bbb_rc.h"

// Global Variables
//static rc_mpu_data_t data;
double servo_rudder_pos =0;
double direction_rudder =1;
double sweep_limit =1.5;
int frequency_hz =50;

double bearing = 90.0; //target direction
double bearing_tolerance = 3.0;
/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int init_bbb_rc(rc_mpu_data_t *data)
{
	// read adc to make sure battery is connected
	if(rc_adc_init()){
		fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
		return -1;
	}
	if(rc_adc_batt()<6.0){
		fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive servos\n");
		return -1;
	}
	printf("bat. voltage=%lf\n", rc_adc_batt());
	rc_adc_cleanup();
	
	// initialize PRU
	if(rc_servo_init()) return -1;
	// turn on power
	printf("Turning On 6V Servo Power Rail\n");
	rc_servo_power_rail_en(1);

	// initialize and configure MPU
	rc_mpu_config_t conf_MPU = rc_mpu_default_config();
	conf_MPU.i2c_bus = I2C_BUS;
	conf_MPU.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
	conf_MPU.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	
	// use in DMP mode
	conf_MPU.dmp_sample_rate = 4; // sample rate in hertz, 200,100,50,40,25,20,10,8,5,4
	conf_MPU.dmp_fetch_accel_gyro = 1;
	conf_MPU.enable_magnetometer = 1;
	//conf_MPU.read_mag_after_callback = 1; //default 1 to improve latency
	//conf_MPU.dmp_interrupt_priority = 1;
	//conf_MPU.dmp_interrupt_sched_policy = SCHED_FIFO;
	conf_MPU.orient= ORIENTATION_Y_UP; // ORIENTATION_Z_UP, ORIENTATION_Z_DOWN, ORIENTATION_X_UP, ORIENTATION_X_DOWN, ORIENTATION_Y_UP, ORIENTATION_Y_DOWN, ORIENTATION_X_FORWARD, ORIENTATION_X_BACK 
	
	// initialize the imu for dmp operation
	if(rc_mpu_initialize_dmp(data, conf_MPU)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}
	//rc_mpu_set_dmp_callback(&print_MPU_data);  // set up the imu for dmp interrupt operation
	//print_header();
	// always sleep at some point
	//rc_usleep(100000);

	return 0;
}

int print_and_log_mpu(rc_mpu_data_t *data)
{
	static int print_header = 1;
	FILE *log_file;
    if ((log_file = fopen("log_bjva_mpu.txt","a")) == NULL)
    {
        printf("\nError creating/opening log file\n");
        return -1;
    }
	if (print_header)
	{
		print_header = 0;
		//printf("\nPress and release pause button to turn green LED on and off");
		//printf("\nhold pause button down for 2 seconds to exit");
		//printf("\nif Mag doesn't work, then recalibrate it");
		//printf("\nNew Passage!");
		//printf("\ncalibrated GYRO-MAG-ACCEL: %d, %d, %d", rc_mpu_is_gyro_calibrated(), rc_mpu_is_mag_calibrated(), rc_mpu_is_accel_calibrated());
		// Header for DMP data
		printf("\n");
		printf(" Raw Compass |");
		printf("FilteredComp|");	
		printf("   Fused Quaternion  |");
		printf("    DMP Quaternion   |");
		printf(" FusedTaitBryan(deg) |");	
		printf(" DMP TaitBryan (deg) |");	
		printf(" Accel XYZ (m/s^2) |");
		printf("  Gyro XYZ (deg/s) |");
		printf(" Temp(C)|");
		printf("\n");

		fprintf(log_file, "\n");
		fprintf(log_file, " Raw Compass |");
		fprintf(log_file, "FilteredComp|");	
		fprintf(log_file, "   Fused Quaternion  |");
		fprintf(log_file, "    DMP Quaternion   |");
		fprintf(log_file, " FusedTaitBryan(deg) |");	
		fprintf(log_file, " DMP TaitBryan (deg) |");	
		fprintf(log_file, " Accel XYZ (m/s^2) |");
		fprintf(log_file, "  Gyro XYZ (deg/s) |");
		fprintf(log_file, " Temp(C)|");
		fprintf(log_file, "\n");		
	}

	printf("    %6.1f   |", data->compass_heading_raw*RAD_TO_DEG);
	printf("   %6.1f   |", data->compass_heading*RAD_TO_DEG);
	printf(" %4.1f %4.1f %4.1f %4.1f |", data->fused_quat[QUAT_W], data->fused_quat[QUAT_X], data->fused_quat[QUAT_Y], data->fused_quat[QUAT_Z]);
	printf(" %4.1f %4.1f %4.1f %4.1f |", data->dmp_quat[QUAT_W], data->dmp_quat[QUAT_X], data->dmp_quat[QUAT_Y], data->dmp_quat[QUAT_Z]);
	printf("%6.1f %6.1f %6.1f |", data->fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG, data->fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG, data->fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	printf("%6.1f %6.1f %6.1f |", data->dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG, data->dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG, data->dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	printf(" %5.2f %5.2f %5.2f |", data->accel[0], data->accel[1], data->accel[2]);
	printf(" %5.1f %5.1f %5.1f |", data->gyro[0], data->gyro[1], data->gyro[2]);
	rc_mpu_read_temp(data);
	printf(" %6.2f |", data->temp);
	printf("\n");

	fprintf(log_file, "    %6.1f   |", data->compass_heading_raw*RAD_TO_DEG);
	fprintf(log_file, "   %6.1f   |", data->compass_heading*RAD_TO_DEG);
	fprintf(log_file, " %4.1f %4.1f %4.1f %4.1f |", data->fused_quat[QUAT_W], data->fused_quat[QUAT_X], data->fused_quat[QUAT_Y], data->fused_quat[QUAT_Z]);
	fprintf(log_file, " %4.1f %4.1f %4.1f %4.1f |", data->dmp_quat[QUAT_W], data->dmp_quat[QUAT_X], data->dmp_quat[QUAT_Y], data->dmp_quat[QUAT_Z]);
	fprintf(log_file, "%6.1f %6.1f %6.1f |", data->fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG, data->fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG, data->fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	fprintf(log_file, "%6.1f %6.1f %6.1f |", data->dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG, data->dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG, data->dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	fprintf(log_file, " %5.2f %5.2f %5.2f |", data->accel[0], data->accel[1], data->accel[2]);
	fprintf(log_file, " %5.1f %5.1f %5.1f |", data->gyro[0], data->gyro[1], data->gyro[2]);
	fprintf(log_file, " %6.2f |", data->temp);
	fprintf(log_file, "\n");

	fclose(log_file);
	return 0;
}

int clean_bbb_rc()
{
	// turn off things and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_mpu_power_off();
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	return 0;
}

int steer_to_bearing(rc_mpu_data_t *data)
{
	//	rc_led_set(RC_LED_GREEN, 1);
	//	rc_led_set(RC_LED_RED, 0);
	if(abs(bearing - data->compass_heading*RAD_TO_DEG) > bearing_tolerance)
	{
		if(bearing < data->compass_heading*RAD_TO_DEG){
			direction_rudder =-1;
		}
		else{
			direction_rudder =1;
		}
		servo_rudder_pos += direction_rudder * sweep_limit / frequency_hz;
		// reset pulse width at end of sweep
		if(servo_rudder_pos>sweep_limit){
			servo_rudder_pos = sweep_limit;
		}
		else if(servo_rudder_pos < (-sweep_limit)){
			servo_rudder_pos = -sweep_limit;
		}
		// send result
		if(rc_servo_send_pulse_normalized(SERVO_RUDDER,servo_rudder_pos)==-1)
		{
			printf("Error servo control\n");
			return -1;
		}
	}
	return 0;
}