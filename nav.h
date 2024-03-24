/**
 * 
 *
 *
 *
 */
#include <stdlib.h>
#include <stdio.h>


#define TERRITORY_CODE 19  // FID 523
#define WAYPOINT_NUMBER_MAX 1024

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
int projection_xy(Waypoint*, Waypoint*);
double distance_between_latlon(struct Coordinate, struct Coordinate);

int distance_between_lat(struct Coordinate*, struct Coordinate*);
int distance_between_lon(struct Coordinate*, struct Coordinate*);

int distance_to_waypoint ();
int heading_to_waypoint (Waypoint*);

int check_safety_zone ();


