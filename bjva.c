#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "nav.c"
//#include "nav.h"
#include "minmea.c"
//#include "minmea.h"
#include "bbb_rc.c"
//#include "bbb_rc.h"

#define TRIG_DISTANCE 0.1

int main()
{
	printf("\nProjet BJ - Voilier Miniature Autonome\n");
	Waypoint_list *waypoint_passage;
	GPS_data gps = {0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, false};
	time_t time_passage = time(NULL);

	init_bbb_rc(); // Init sensors : button, ADC, MPU and servos
	waypoint_passage = read_waypoint_file();

	printf("\nStart Passage, time 0= %lds ", time_passage);
	do
	{
		get_gps_coordinate(&gps);
		if(goto_next_waypoint(waypoint_passage, &gps) < TRIG_DISTANCE)
		{
			waypoint_passage->target_waypoint = waypoint_passage->target_waypoint->next_waypoint;
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

