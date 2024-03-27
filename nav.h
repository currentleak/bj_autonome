/**
 * 
 *
 *
 *
 */
#include <stdlib.h>
#include <stdio.h>


//#define TERRITORY_CODE 19  // FID 523
#define DEG2RAD 0.0174532925199 // Pi/180
#define EARTH_RADIUS 6371

struct Coordinate
{
    double latitude;
    double longitude; 
};

struct Point
{
    int x;
    int y;
};

typedef struct Waypoint // latitude-longitude telling us where to go
{
    struct Coordinate wp_coordinate;
    struct Point xy_coord;
    int identification;
    int target_radius;
    double distance_to_next_wp;
    double bearing_to_next_wp;
    struct Waypoint* next_waypoint;
    struct Waypoint* prev_waypoint;
}Waypoint;

typedef struct Waypoint_list // waypoint list for passage
{
    Waypoint* first_waypoint;
    Waypoint* destination_waypoint;
    int waypoint_qty;
}Waypoint_list;

Waypoint_list* read_waypoint_file();
double calculate_distance(struct Coordinate, struct Coordinate);
double calculate_bearing(struct Coordinate, struct Coordinate);
void destroy_waypoint_list(Waypoint_list*);

//int check_safety_zone ();
//int projection_xy(Waypoint*, Waypoint*);



