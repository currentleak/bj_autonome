/**
 * 
 *
 *
 *
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <rc_usefulincludes.h>
#include <rc/mpu.h>
#include <rc/time.h>

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

#define SERVO_RUDDER 1  // servo for rudder
#define SERVO_SHEET 2   // servo for main sail and genova sheet


// function declarations
int init_bbb_rc();
void on_pause_press();
void on_pause_release();

static void print_header();
static void print_MPU_data();






