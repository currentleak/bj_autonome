//#include <stdlib.h>
#include <stdio.h>
//#include <unistd.h>
#include <time.h>

#include "nav.h"
#include "minmea.h"
#include "bbb_rc.h"

int main()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");
	Waypoint_list *waypoint_passage;
	GPS_data gps = {0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, false, {0.0, 0.0}};
	time_t time_passage;
	double distance = 0.0;

	FILE *log_file;
    if ((log_file = fopen("log_bjva.txt","a")) == NULL)
    {
        printf("\nError creating log file\n");
        return -1;
    }

	//init_bbb_rc(); // Init sensors : button, ADC, MPU and servos
	
	waypoint_passage = read_waypoint_file();
	if(waypoint_passage==NULL)
		return -1;
	print_WP_list(waypoint_passage);
	waypoint_passage->target_waypoint = waypoint_passage->first_waypoint; // GPS coord Starting point
	if(waypoint_passage->target_waypoint==NULL)
		return -1;

    printf("\nWaiting to be at starting line... \n");

	do
	{
		sleep(1);
		get_gps_coordinate(&gps);
		distance = goto_next_waypoint(waypoint_passage, &gps);
		printf("\rNext waypoint Id= %3d, Distance to next waypoint= %8.3lf, Bearing= %5.1lf, ", waypoint_passage->target_waypoint->identification, distance, 
				calculate_bearing(&(gps.gps_coord), waypoint_passage->target_waypoint->wp_coordinate));
		fprintf(log_file, "\nNext waypoint Id= %3d, Distance to next waypoint= %8.3lf, Bearing= %5.1lf, ", waypoint_passage->target_waypoint->identification, distance, 
				calculate_bearing(&(gps.gps_coord), waypoint_passage->target_waypoint->wp_coordinate));
		fflush(log_file);
		log_GPS_data(&gps, log_file);		
		print_GPS_data(&gps);

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

	fclose(log_file);
	fflush(stdout);
	if(waypoint_passage!=NULL) {
		destroy_waypoint_list(waypoint_passage);
	}
	return 0;
}

