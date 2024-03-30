#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "nav.c"
//#include "nav.h"

int main()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");

	// Init GPS


	// Init sensors : button, ADC, MPU and servos


	Waypoint_list *waypoint_passage = read_waypoint_file();
	Waypoint *wp_current;

	if(waypoint_passage==NULL)
	{
		printf("\nError no passage defined");
	}
	else
	{
		printf("\nwaypoint qty=%d", waypoint_passage->waypoint_qty);
		printf("\nWP#   Latitude   Longitude  Next WP: distance, bearing");
		wp_current = waypoint_passage->first_waypoint;
		double dist = 0.0;
		while (wp_current != NULL)
    	{
			printf("\n%3d  ", wp_current->identification);
        	printf(" %lf  %lf   ", wp_current->wp_coordinate->latitude, wp_current->wp_coordinate->longitude);
			printf("     %8.3lf, %5.1lf", wp_current->distance_to_next_wp, wp_current->bearing_to_next_wp);
			dist = dist + wp_current->distance_to_next_wp;
        	wp_current = wp_current->next_waypoint;
    	}
    	printf("\nDistance totale= %8.3lf\n", dist);
	}
	
	wp_current = waypoint_passage->first_waypoint; // GPS coord Starting point

	// get locked GPS

	Coordinate (*coord_current) = malloc(sizeof(*coord_current));
	if(coord_current==NULL)
	{    
        printf("\nError allocating memory"); 
        return 1;
    
	}
	printf("\nWaiting to be at starting line...");
	double active_distance = 0.0;
	do
	{
		get_gps_coordinate(coord_current);
		active_distance = calculate_distance(coord_current, wp_current->wp_coordinate);
		printf("\nDistance to starting line = %lf", active_distance);
		fflush(stdout);
		sleep(5);
	} while (active_distance > 10.0);
	

	while(wp_current != waypoint_passage->destination_waypoint)
	{
		//goto_waypoint(wp_current);
	}
	

	fflush(stdout);
	if(waypoint_passage!=NULL)
	{
		destroy_waypoint_list(waypoint_passage);
	}
	free(coord_current);
	return 0;
}

