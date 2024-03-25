#include <stdlib.h>
#include <stdio.h>

#include "nav.c"
//#include "nav.h"

int main()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");

	Waypoint_list *waypoint_passage = read_waypoint_file();
	if(waypoint_passage==NULL)
	{
		printf("\nError no passage defined");
	}
	else
	{
		printf("\nwaypoint qty=%d", waypoint_passage->waypoint_qty);
		printf("\nWP#   Latitude   Longitude  Next WP: distance, bearing");
		Waypoint* actuel = waypoint_passage->first_waypoint;
		while (actuel != NULL)
    	{
			printf("\n%3d  ", actuel->identification);
        	printf(" %lf  %lf   ", actuel->wp_coordinate.latitude, actuel->wp_coordinate.longitude);
			printf("     %8.3lf, %5.1lf", actuel->distance_to_next_wp, actuel->bearing_to_next_wp);
        	actuel = actuel->next_waypoint;
    	}
    	printf("\n");
	}

}

