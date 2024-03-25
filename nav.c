/**
 * 
 *
 *
 *
 */
#include "nav.h"
#include <math.h>

Waypoint_list* read_waypoint_file()
{
    // read file to fill the waypoint list
    FILE* fptr;
    double lat, lon;
    if ((fptr = fopen("waypoint_list.dat","r")) == NULL)
    {
        printf("\nError reading waypoints list file");                
        return NULL;
    }
    // init waypoint list and create first waypoint
    Waypoint_list* wp_list = malloc(sizeof(* wp_list));
    Waypoint* wp = malloc(sizeof(* wp));
    if(wp_list==NULL || wp==NULL)
    {
        printf("\nError allocating memory"); 
        fclose(fptr);
        return NULL;
    }
    if(fscanf(fptr,"%lf %lf", &lat, &lon) != EOF)
    {
        //printf("\nlat=%lf, lon=%lf", lat, lon);
        wp->wp_coordinate.latitude = lat;
        wp->wp_coordinate.longitude = lon;
        wp->identification = 0;  // starting waypoint
        wp->target_radius = 1;
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
        if(wp==NULL)
        {
            printf("\nError allocating memory"); 
            fclose(fptr);
            return NULL;
        }
        wp->wp_coordinate.latitude = lat;
        wp->wp_coordinate.longitude = lon;
        wp->identification = i;
        wp->target_radius = 10;
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
    return wp_list;
}

double calculate_distance(struct Coordinate c1, struct Coordinate c2)
{
    double term1 = pow(sin((DEG2RAD*(c2.latitude - c1.latitude))/ 2), 2);
    double term2 = cos(DEG2RAD*c1.latitude) * cos(DEG2RAD*c2.latitude);
    double term3 = pow(sin((DEG2RAD*(c2.longitude - c1.longitude))/ 2), 2);
    return 2*EARTH_RADIUS * asin(sqrt(term1 + (term2 * term3))); 
/*     double dlat = DEG2RAD*(c2.latitude - c1.latitude);
    double dlon = DEG2RAD*(c2.longitude - c1.longitude);
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(DEG2RAD*c1.latitude) * cos(DEG2RAD*c2.latitude) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return = EARTH_RADIUS * c;   */
}

// Function to calculate initial bearing between two points
double calculate_bearing(struct Coordinate c1, struct Coordinate c2) {
    double delta_lon = DEG2RAD*(c2.longitude - c1.longitude);
    double x = cos(DEG2RAD*c2.latitude) * sin(delta_lon);
    double y = cos(DEG2RAD*c1.latitude) * sin(DEG2RAD*c2.latitude) - sin(DEG2RAD*c1.latitude) * cos(DEG2RAD*c2.latitude) * cos(delta_lon);
    double bearing = atan2(x, y);
    bearing = bearing/DEG2RAD;
    // Adjusting bearing to be in the range of [0, 360) degrees
    if (bearing < 0) {
        bearing += 360.0;
    }
    return bearing;
}

