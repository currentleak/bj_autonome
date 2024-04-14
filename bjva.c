/**
 * 
 *
 *
 *
 */
//#include <stdlib.h>
#include <stdio.h>
//#include <unistd.h>
#include <time.h>

#include "nav.h"
#include "minmea.h"
#include "bbb_rc.h"

void clean_mem(Waypoint_list *);

int main()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");
	time_t time_passage;
	rc_mpu_data_t data_mpu;
	GPS_data gps = {0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, false, {0.0, 0.0}};
	double distance = 0.0;
	Waypoint_list *waypoint_passage = read_waypoint_file();

	if(waypoint_passage == NULL || waypoint_passage->first_waypoint == NULL)
	{
		printf("\nError no passagedefined \n");
		return -1;
	}
	print_WP_list(waypoint_passage);
	waypoint_passage->target_waypoint = waypoint_passage->first_waypoint; // GPS coord Starting point

	if(init_bbb_rc(&data_mpu)!=0) // Init sensors : button, ADC, MPU and servos
	{
		printf("\nError init MPU\n");
		clean_mem(waypoint_passage);
		return -1;
	}

    printf("\nWaiting to be at starting line... \n");
	do
	{	// TODO : check if the following next waypoint is nearest than the next waypoint
		sleep(1);
		get_gps_coordinate(&gps);
		distance = goto_waypoint(waypoint_passage, &gps);
		print_and_log_nav(waypoint_passage, &gps, distance);
		print_and_log_mpu(&data_mpu);
		printf("\033[F");
		printf("\033[F");

		if( distance < waypoint_passage->target_waypoint->target_radius)
		{
			if(waypoint_passage->target_waypoint == waypoint_passage->first_waypoint)
			{
				time_passage = time(NULL);
				printf("\nStart Passage, time 0= %lds", time_passage);
			}
			waypoint_passage->target_waypoint = waypoint_passage->target_waypoint->next_waypoint;
			printf("\n");
		}
	} while (waypoint_passage->target_waypoint != NULL);
	
	time_passage = time(NULL) - time_passage;
	printf("\nAt destination! Duration= %lds\n\n", time_passage);

	clean_mem(waypoint_passage);
	return 0;
}

void clean_mem(Waypoint_list *waypoint_passage)
{
	clean_bbb_rc();
	fflush(stdout);
	if(waypoint_passage!=NULL) {
		destroy_waypoint_list(waypoint_passage);
	}
}