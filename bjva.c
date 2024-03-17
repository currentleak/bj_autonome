/**
 * @file rc_project_bjva.c
 *
 * program for Robot Control projects. 
 */

#include "bjva.h"

// Global Variables
uint64_t epoch_start = 0;
static rc_mpu_data_t data;

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
int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1)
	{
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
	{
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
	
	epoch_start = rc_nanos_since_epoch();
	print_header();

	// initialize and configure MPU
	rc_mpu_config_t conf_MPU = rc_mpu_default_config();
	conf_MPU.i2c_bus = I2C_BUS;
	conf_MPU.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
	conf_MPU.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	
	// use in DMP mode
	conf_MPU.dmp_sample_rate = 4; // sample rate in hertz, 200,100,50,40,25,20,10,8,5,4
	conf_MPU.dmp_fetch_accel_gyro = 1;
	conf_MPU.enable_magnetometer = 1;
	conf_MPU.read_mag_after_callback = 1; //default 1 to improve latency
	//conf_MPU.dmp_interrupt_priority = 1;
	//conf_MPU.dmp_interrupt_sched_policy = SCHED_FIFO;
	conf_MPU.orient= ORIENTATION_Z_UP;
	
	// initialize the imu for dmp operation
	if(rc_mpu_initialize_dmp(&data, conf_MPU)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}
	
	rc_mpu_set_dmp_callback(&print_MPU_data);  // set up the imu for dmp interrupt operation
	
	rc_set_state(RUNNING);
	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
						
		}
		else{
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		// always sleep at some point
		rc_usleep(100000);
	}

	// turn off things and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_mpu_power_off();
	printf("\n\n");
	printf("total time: %llu\n", rc_nanos_since_epoch()-epoch_start);
	fflush(stdout);
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}

/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	printf("time: %llu\n", rc_nanos_since_epoch()-epoch_start);
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("\n\nlong press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

void print_header()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");
	printf("\nPress and release pause button to turn green LED on and off");
	printf("\nhold pause button down for 2 seconds to exit\n");
	printf("calibrated GYRO-MAG-ACCEL: %d, %d, %d\n", rc_mpu_is_gyro_calibrated(), rc_mpu_is_mag_calibrated(), rc_mpu_is_accel_calibrated());
	printf("\nstart time: %llu", epoch_start);
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
}

void print_MPU_data()
{
	rc_mpu_read_mag(&data);
	printf("\r");
	printf("    %6.1f   |", data.compass_heading_raw*RAD_TO_DEG);
	printf("   %6.1f   |", data.compass_heading*RAD_TO_DEG);
	printf(" %4.1f %4.1f %4.1f %4.1f |",	data.fused_quat[QUAT_W], data.fused_quat[QUAT_X], data.fused_quat[QUAT_Y], data.fused_quat[QUAT_Z]);
	printf(" %4.1f %4.1f %4.1f %4.1f |",	data.dmp_quat[QUAT_W], data.dmp_quat[QUAT_X], data.dmp_quat[QUAT_Y], data.dmp_quat[QUAT_Z]);
	printf("%6.1f %6.1f %6.1f |",	data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG, data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG, data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	printf("%6.1f %6.1f %6.1f |",	data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG, data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG, data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	printf(" %5.2f %5.2f %5.2f |",	data.accel[0], data.accel[1], data.accel[2]);
	printf(" %5.1f %5.1f %5.1f |",	data.gyro[0], data.gyro[1], data.gyro[2]);
	rc_mpu_read_temp(&data);
	printf(" %6.2f |", data.temp);
	fflush(stdout);
	
}