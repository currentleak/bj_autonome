/**
 * @file    nav.h
 * @brief   Handle navigation data
 *
 * @author  Kevin Cotton
 * @date    2024-07-14
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "minmea.h"

//#define TERRITORY_CODE 19  // FID 523
#define DEG2RAD 0.0174532925199 // Pi/180
#define EARTH_RADIUS 6371 // km

#define INDENT_SPACES "  "

typedef struct Coordinate ///< Coordinate: position on Earth (lat, long)
{
    double latitude;
    double longitude; 
}Coordinate;

typedef struct Waypoint ///< latitude-longitude telling us where to go
{
    struct Coordinate *wp_coordinate;
    int identification;
    double target_radius;
    double distance_to_next_wp;
    double bearing_to_next_wp;
    struct Waypoint *next_waypoint;
    struct Waypoint *prev_waypoint;
}Waypoint;

typedef struct Waypoint_list ///< waypoint list for passage
{
    Waypoint *first_waypoint;
    Waypoint *destination_waypoint;
    Waypoint *target_waypoint;
    int waypoint_qty;
}Waypoint_list;

typedef struct GPS_data ///< GPS data and status
{
    int fix_quality;
    int sat_in_view;
    float true_track, declin_mag;
    float speed_kph;
    int hour, minute, second, day, month, year;
    int h_offset, m_offset;
    bool valid;
    Coordinate gps_coord;
}GPS_data;

Waypoint_list *read_waypoint_file(void); ///< read waypoint from a pre-existing file
int print_WP_list(Waypoint_list *); ///< Print the Waypoint list to the console
void destroy_waypoint_list(Waypoint_list *); ///< clean waypoint memory
double calculate_distance(Coordinate *, Coordinate *); ///< Compute distance between 2 coordinates
double calculate_bearing(Coordinate *, Coordinate *); ///< Compute angle from a coordinate to another coordinate

int get_gps_coordinate(GPS_data *);
double goto_waypoint(Waypoint_list *, GPS_data *);

int print_and_log_nav(Waypoint_list *, GPS_data *, double);


//int check_safety_zone ();

