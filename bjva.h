/**
 * 
 *
 *
 *
 */
#define _GNU_SOURCE

#include <stdio.h>
//#include <getopt.h>
//#include <signal.h>
//#include <stdlib.h> // for atoi() and exit()
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <rc_usefulincludes.h>
#include <rc/mpu.h>
#include <rc/time.h>

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// function declarations
void on_pause_press();
void on_pause_release();

void print_header();
void print_MPU_data();






