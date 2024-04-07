//#include <stdlib.h>
#include <stdio.h>
//#include <unistd.h>
#include <time.h>

#include "nav.h"
#include "minmea.h"
#include "bbb_rc.h"

int print_log(Waypoint_list *, GPS_data *, double distance);

int main()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");
	time_t time_passage;
	GPS_data gps = {0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, false, {0.0, 0.0}};
	double distance = 0.0;
	Waypoint_list *waypoint_passage = read_waypoint_file();

	if(waypoint_passage==NULL)
	{
		printf("\nError no passage\n");
		return -1;
	}
	print_WP_list(waypoint_passage);
	waypoint_passage->target_waypoint = waypoint_passage->first_waypoint; // GPS coord Starting point
	if(waypoint_passage->target_waypoint==NULL)
	{
		printf("\nError no waypoint\n");
		return -1;
	}

	//init_bbb_rc(); // Init sensors : button, ADC, MPU and servos


    printf("\nWaiting to be at starting line... \n");
	do
	{	// TODO : check if the following next waypoint is nearest than the next waypoint
		sleep(1);
		get_gps_coordinate(&gps);
		distance = goto_next_waypoint(waypoint_passage, &gps);
		print_log(waypoint_passage, &gps, distance);
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

	fflush(stdout);
	if(waypoint_passage!=NULL) {
		destroy_waypoint_list(waypoint_passage);
	}
	return 0;
}

int print_log(Waypoint_list *wpl, GPS_data *gps, double d)
{
	static int try = 0;
	FILE *log_file;
    if ((log_file = fopen("log_bjva.txt","a")) == NULL)
    {
        printf("\nError creating/opening log file\n");
        return -1;
    }
	if (try==0)
	{
		try=1;
		printf("\nNew Passage!");
		printf("\nWP ID,  Distance to WP, Bearing, GPS Qty, Fix Qual,   latitude,  longitude,       time,     date,  course,   speed\n");
		fprintf(log_file, "\nNew Passage!");
		fprintf(log_file, "\nWP ID,  Distance to WP, Bearing, GPS Qty, Fix Qual,   latitude,  longitude,       time,     date,  course,   speed\n");
	}
	printf("\r  %3d,        %8.3lf,   %5.1lf, ", wpl->target_waypoint->identification, d, 
				calculate_bearing(&(gps->gps_coord), wpl->target_waypoint->wp_coordinate));
	fprintf(log_file, "\n  %3d,        %8.3lf,   %5.1lf, ", wpl->target_waypoint->identification, d, 
				calculate_bearing(&(gps->gps_coord), wpl->target_waypoint->wp_coordinate));

	log_GPS_data(gps, log_file);		
	print_GPS_data(gps);

	fflush(stdout);
	fflush(log_file);

	fclose(log_file);
	return 0;
}