/**
 * 
 *
 *
 *
 */
#include "nav.h"

Waypoint_list *read_waypoint_file()
{
    // read file to fill the waypoint list
    FILE *fptr;
    double lat, lon;
    if ((fptr = fopen("waypoint_list.dat","r")) == NULL)
    {
        printf("\nError reading waypoints list file");                
        return NULL;
    }
    // init waypoint list and create first waypoint
    Waypoint_list (*wp_list) = malloc(sizeof(*wp_list));
    Waypoint (*wp) = malloc(sizeof(*wp));
    Coordinate (*coord) = malloc(sizeof(*coord));
    if(wp_list==NULL || wp==NULL || coord==NULL)
    {
        printf("\nError allocating memory"); 
        fclose(fptr);
        return NULL;
    }
    wp->wp_coordinate = coord;
    if(fscanf(fptr,"%lf %lf", &lat, &lon) != EOF)
    {
        //printf("\nlat=%lf, lon=%lf", lat, lon);
        wp->wp_coordinate->latitude = lat;
        wp->wp_coordinate->longitude = lon;
        wp->identification = 0;  // starting waypoint
        wp->target_radius = 0.01; // 10m
        wp->next_waypoint = NULL;
        wp->prev_waypoint = NULL;
        wp_list->first_waypoint = wp;
        wp_list->destination_waypoint = wp;
        wp_list->waypoint_qty = 1;
    } 
    else
    {
        printf("\nError waypoints list file is empty"); 
        fclose(fptr);
        return NULL;
    }
    // add all other waypoints
    int i=1;
    while (fscanf(fptr,"%lf %lf", &lat, &lon) != EOF)
    {
        //printf("\nlat=%lf, lon=%lf", lat, lon);
        wp = malloc(sizeof(* wp));
        coord = malloc(sizeof(*coord));
        if(wp==NULL || coord==NULL)
        {
            printf("\nError allocating memory"); 
            fclose(fptr);
            return NULL;
        }
        wp->wp_coordinate = coord;
        wp->wp_coordinate->latitude = lat;
        wp->wp_coordinate->longitude = lon;
        wp->identification = i;
        wp->target_radius = 0.1;  // 100m
        wp->prev_waypoint = wp_list->destination_waypoint;
        wp->next_waypoint = NULL;
        wp->distance_to_next_wp = 0.0;
        wp->bearing_to_next_wp = 0.0;
        wp_list->destination_waypoint = wp;    
        wp->prev_waypoint->next_waypoint = wp; 
        wp->prev_waypoint->distance_to_next_wp = calculate_distance(wp->prev_waypoint->wp_coordinate, wp->wp_coordinate);
        wp->prev_waypoint->bearing_to_next_wp =  calculate_bearing(wp->prev_waypoint->wp_coordinate, wp->wp_coordinate);  
        wp_list->waypoint_qty++;
        i++;
    }
    fclose(fptr);
    wp->target_radius = 0.01; // 10m, destination waypoint has a closer target radius
    if(wp_list->waypoint_qty < 2)
    {
        printf("\nWaypoint list should contain at least 2 coordinates to make a passage");
        return NULL;
    }
    return wp_list;
}

double calculate_distance(Coordinate *c1, Coordinate *c2)
{
    double term1 = pow(sin((DEG2RAD*(c2->latitude - c1->latitude))/ 2), 2);
    double term2 = cos(DEG2RAD*c1->latitude) * cos(DEG2RAD*c2->latitude);
    double term3 = pow(sin((DEG2RAD*(c2->longitude - c1->longitude))/ 2), 2);
    return 2*EARTH_RADIUS * asin(sqrt(term1 + (term2 * term3))); 
/*     double dlat = DEG2RAD*(c2->latitude - c1->latitude);
    double dlon = DEG2RAD*(c2->longitude - c1->longitude);
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(DEG2RAD*c1->latitude) * cos(DEG2RAD*c2->latitude) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return = EARTH_RADIUS * c;   */
}

// Function to calculate initial bearing between two points
double calculate_bearing(Coordinate *c1, Coordinate *c2) 
{
    double delta_lon = DEG2RAD*(c2->longitude - c1->longitude);
    double x = cos(DEG2RAD*c2->latitude) * sin(delta_lon);
    double y = cos(DEG2RAD*c1->latitude) * sin(DEG2RAD*c2->latitude) - sin(DEG2RAD*c1->latitude) * cos(DEG2RAD*c2->latitude) * cos(delta_lon);
    double bearing = atan2(x, y);
    bearing = bearing/DEG2RAD;
    // Adjusting bearing to be in the range of [0, 360) degrees
    if (bearing < 0) {
        bearing += 360.0;
    }
    return bearing;
}

void destroy_waypoint_list(Waypoint_list *wp_list)
{
	Waypoint *wp, *wp_next;
	wp = wp_list->first_waypoint;
	while(wp!=NULL)
	{
		wp_next = wp->next_waypoint;
        free(wp->wp_coordinate);
		free(wp);
		wp = wp_next;
	}
	free(wp_list);
}

int get_gps_coordinate(Coordinate *gps_coord)
{
    //todo: check if gps is locked 

    //data for simulation
    static double c[30]={46.3, -71.5, 46.5, -71.7, 46.68, -71.87, 46.6839, -71.8785, 46.68, -71.83, 46.685, -71.838, 46.68576, -71.83897, 
    46.668, -71.791, 46.668241, -71.791758, 46.666, -71.693, 46.666564, -71.693272, 46.673, -71.665, 46.673956, -71.665416, 46.697, -71.576, 46.69703, -71.57623};
    //46.683924  -71.878586 --> start
    //46.685768 -71.838973 --> wp1
    //46.668241 -71.791758
    //46.666564 -71.693272
    //46.673956 -71.665416
    //46.697037 -71.576235 --> destination
    static int i=0;
    gps_coord->latitude = c[i];
    gps_coord->longitude = c[i+1];
    i=i+2;
    if(i==30) i=0;
    return 0;

}

double goto_waypoint(Coordinate *coord, Waypoint *wp)
{
    //todo: PID goto wp

    return calculate_distance(coord, wp->wp_coordinate);
}