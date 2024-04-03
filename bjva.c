#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "nav.c"
//#include "nav.h"
#include "minmea.c"
//#include "bbb_rc.c"
//#include "bbb_rc.h"

int main()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");

	// Init GPS
	// Init sensors : button, ADC, MPU and servos

	Waypoint_list *waypoint_passage = read_waypoint_file();
	Waypoint *target_wp;
	double distance_to_target = 0.0;
	double bearing_to_target = 0.0;
	time_t time_passage;
	GPS_data gps;

	if(waypoint_passage==NULL)
	{
		printf("\nError no passage defined");
		return 1;
	}
	else
	{
		printf("\nwaypoint qty=%d", waypoint_passage->waypoint_qty-1); //excluding the starting waypoint
		printf("\nWP#   Latitude   Longitude  Next WP: distance, bearing");
		target_wp = waypoint_passage->first_waypoint;
		while (target_wp != NULL)
    	{
			printf("\n%3d  ", target_wp->identification);
        	printf(" %lf  %lf   ", target_wp->wp_coordinate->latitude, target_wp->wp_coordinate->longitude);
			printf("     %8.3lf, %5.1lf", target_wp->distance_to_next_wp, target_wp->bearing_to_next_wp);
			distance_to_target = distance_to_target + target_wp->distance_to_next_wp;
        	target_wp = target_wp->next_waypoint;
    	}
    	printf("\nDistance totale= %8.3lf\n", distance_to_target);
	}
	
	target_wp = waypoint_passage->first_waypoint; // GPS coord Starting point

	printf("\nWaiting to be at starting line... \n");
	do
	{
		get_gps_coordinate(&gps);
		printf("\rGPS Qty:%3d, Fix Qual:%2d, ", gps.sat_in_view, gps.fix_quality);
		distance_to_target = calculate_distance(&gps.gps_coord , target_wp->wp_coordinate);
		printf("Distance to starting line = %8.3lf", distance_to_target);
		printf("lat: %lf, lon:%lf", gps.gps_coord.latitude, gps.gps_coord.longitude);
		fflush(stdout);
		sleep(1);
	} while (isnan(distance_to_target) || distance_to_target > 0.01);

	target_wp = target_wp->next_waypoint;
	time_passage = time(NULL);
	
	printf("\nStart passage... time= %ld\n", time_passage);
	do
	{
		get_gps_coordinate(&gps);
		distance_to_target = goto_waypoint(&gps.gps_coord, target_wp);
		bearing_to_target = calculate_bearing(&gps.gps_coord, target_wp->wp_coordinate);
		printf("\rNext waypoint Id= %3d, Distance to next waypoint= %8.3lf, Bearing= %5.1lf",target_wp->identification, distance_to_target, bearing_to_target);
		fflush(stdout);
		if(!isnan(distance_to_target) && distance_to_target < target_wp->target_radius)
		{
			target_wp = target_wp->next_waypoint;
			printf("\n");
		}
		sleep(1);
	}
	while(target_wp != waypoint_passage->destination_waypoint);

	time_passage = time(NULL) - time_passage;
	printf("\nAt destination!  Duration= %lds ", time_passage);

	fflush(stdout);
	if(waypoint_passage!=NULL)
	{
		destroy_waypoint_list(waypoint_passage);
	}
	return 0;
}

