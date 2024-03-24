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
        wp_list->destination_waypoint = wp;    
        wp->prev_waypoint->next_waypoint = wp;     
        wp_list->waypoint_qty++;
        i++;
    }
    fclose(fptr);
    return wp_list;
}

int projection_xy(Waypoint *wp1, Waypoint *wp2)
{
    double minLat = 0.0;
    double maxLat = 0.0;
    double minLon = 0.0;
    double maxLon = 0.0;
    double positionX = 0.0;
    double positionY = 0.0;
    int MAP_HEIGHT = 5000;
    int MAP_WIDTH = 20000;

    if(wp1->wp_coordinate.latitude > wp2->wp_coordinate.latitude) // find minimum latitude
        minLat=wp2->wp_coordinate.latitude;
    else
        minLat=wp1->wp_coordinate.latitude;

    if(wp1->wp_coordinate.longitude > wp2->wp_coordinate.longitude) // find minimum longitude
        minLon=wp2->wp_coordinate.longitude;
    else
        minLon=wp1->wp_coordinate.longitude;

    if(wp1->wp_coordinate.latitude < wp2->wp_coordinate.latitude) // find maximum latitude
        maxLat=wp2->wp_coordinate.latitude;
    else
        maxLat=wp1->wp_coordinate.latitude;

    if(wp1->wp_coordinate.longitude < wp2->wp_coordinate.longitude) // find maximum longitude
        maxLon=wp2->wp_coordinate.longitude;
    else
        maxLon=wp1->wp_coordinate.longitude;

    // compute xy for both waypoint
    wp1->xy_coord.y = ((wp1->wp_coordinate.latitude - minLat) / (maxLat - minLat)) * (MAP_HEIGHT - 1);
    wp1->xy_coord.x = ((wp1->wp_coordinate.longitude - minLon) / (maxLon - minLon)) * (MAP_WIDTH - 1);
    wp2->xy_coord.y = ((wp2->wp_coordinate.latitude - minLat) / (maxLat - minLat)) * (MAP_HEIGHT - 1);
    wp2->xy_coord.x = ((wp2->wp_coordinate.longitude - minLon) / (maxLon - minLon)) * (MAP_WIDTH - 1);

}


double distance_between_latlon(struct Coordinate c1, struct Coordinate c2)
{
    // https://www.omnicalculator.com/other/latitude-longitude-distance
    // d = 2R × sin⁻¹(√[sin²((θ₂ - θ₁)/2) + cosθ₁ × cosθ₂ × sin²((φ₂ - φ₁)/2)])
    
    // https://www.movable-type.co.uk/scripts/latlong.html 
    // a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
    // c = 2 ⋅ atan2( √a, √(1−a) )
    // d = R ⋅ c
    
    double distance = 0.0;
    double earth_radius = 6371.0;
    double deg2rad = 0.0174532925199; // Pi/180

    double term1 = pow(sin((deg2rad*c2.latitude - deg2rad*c1.latitude)/ 2), 2);
    double term2 = cos(deg2rad*c1.latitude) * cos(deg2rad*c2.latitude);
    double term3 = pow(sin((deg2rad*c2.longitude - deg2rad*c1.longitude)/ 2), 2);

    distance = 2*earth_radius * asin(sqrt(term1 + (term2 * term3))); 
    return distance;
}