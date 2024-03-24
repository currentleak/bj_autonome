#include <stdlib.h>
#include <stdio.h>

#include "nav.c"
//#include "nav.h"

int main()
{
	Waypoint_list *waypoint_passage = read_waypoint_file();
	if(waypoint_passage==NULL)
	{
		printf("\nError no passage defined");
	}
	else
	{
		printf("\nwaypoint qty=%d", waypoint_passage->waypoint_qty);
		Waypoint* actuel = waypoint_passage->first_waypoint;
		while (actuel != NULL)
    	{
			printf("\n wp#:%3d ", actuel->identification);
        	printf("lat: %lf lon: %lf", actuel->wp_coordinate.latitude, actuel->wp_coordinate.longitude);
        	actuel = actuel->next_waypoint;
    	}
    	printf("\nNULL\n");
	}


	printf("\ndistance= %lf", distance_between_latlon(waypoint_passage->first_waypoint->wp_coordinate, waypoint_passage->destination_waypoint->wp_coordinate));
	printf("\n");
}

