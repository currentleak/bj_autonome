/**
 * @file    nav.c
 * @brief   Handle navigation data
 *
 * @author  Kevin Cotton
 * @date    2024-07-14
 *
 */
#include "nav.h"

/**
 * @brief		Read file to fill the Waypoint list in memory
 * @return		Pointer to the new Waypoint list
 */
Waypoint_list *read_waypoint_file()
{
    FILE *fptr;
    double lat, lon;
    if ((fptr = fopen("waypoint_list.dat","r")) == NULL)
    {
        printf("\nError reading waypoints list file\n");
        return NULL;
    }
    // init waypoint list and create first waypoint
    Waypoint_list (*wp_list) = malloc(sizeof(*wp_list));
    Waypoint (*wp) = malloc(sizeof(*wp));
    Coordinate (*coord) = malloc(sizeof(*coord));
    if(wp_list==NULL || wp==NULL || coord==NULL)
    {
        printf("\nError allocating memory\n");
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
        wp->target_radius = 0.05; // 50m
        wp->next_waypoint = NULL;
        wp->prev_waypoint = NULL;
        wp_list->first_waypoint = wp;
        wp_list->destination_waypoint = wp;
        wp_list->waypoint_qty = 1;
    } 
    else
    {
        printf("\nError waypoints list file is empty\n"); 
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
            printf("\nError allocating memory\n"); 
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
    wp->target_radius = 0.05; // 50m, destination waypoint has a closer target radius
    if(wp_list->waypoint_qty < 2)
    {
        printf("\nError Waypoint list should contain at least 2 coordinates\n");
        return NULL;
    }
    return wp_list;
}

/**
 * @brief		Print the waypoint list and total distance to the console
 * @param		Pointer to the waypoint list 
 * @return		0 during normal operation, -1 on error
 */
int print_WP_list(Waypoint_list *wp_list)
{
    // Print the Waypoint List   
    if(wp_list==NULL)
    {
        return -1;
    } 
	printf("\nwaypoint qty=%d", wp_list->waypoint_qty-1); //excluding the starting waypoint
	printf("\nWP#   Latitude   Longitude  Next WP: distance, bearing");
	wp_list->target_waypoint = wp_list->first_waypoint;
    double distance_to_target = 0.0;
	while (wp_list->target_waypoint != NULL)
    {
		printf("\n%3d  ", wp_list->target_waypoint->identification);
       	printf(" %lf  %lf   ", wp_list->target_waypoint->wp_coordinate->latitude, wp_list->target_waypoint->wp_coordinate->longitude);
		printf("     %8.3lf, %5.1lf", wp_list->target_waypoint->distance_to_next_wp, wp_list->target_waypoint->bearing_to_next_wp);
		distance_to_target = distance_to_target + wp_list->target_waypoint->distance_to_next_wp;
       	wp_list->target_waypoint = wp_list->target_waypoint->next_waypoint;
    }
    printf("\nDistance totale= %8.3lf\n", distance_to_target);
    fflush(stdout);
    return 0;
}

/**
 * @brief		Compute distance between 2 posistions in lat-long coordinate
 * @param		Pointer to structure coordinate : 2 positions 
 * @return		distance in meter
 */
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

/**
 * @brief		Compute bearing between two points coordinate
 * @param		Pointer to structure coordinate : 2 positions 
 * @return		angle (bearing) to next from coord 1 to coord 2
 */
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

/**
 * @brief		Clean waypoint list memory 
 * @param		Pointer waypoint list
 */
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

/**
 * @brief		Get coordinate from GPS Hardware
 * @param		Pointer to structure status and data for GPS 
 * @return		0 during normal operation, -1 on error
 */
int get_gps_coordinate(GPS_data *gps)
{
    char line[MINMEA_MAX_SENTENCE_LENGTH];
    FILE *port_com = fopen("/dev/ttyO1", "r"); // Beagle Blue : ttyO1=Uart1(3.3V), ttyO2=Uart2(GPS 5V)
    if(port_com == NULL)  
    {
        printf("\nError opening GPS serial port!\n");     
        return -1;
    }
    gps->valid = false;
    fflush(port_com);
    int timeout = 0;
    while(fgets(line, sizeof(line), port_com) != NULL && !gps->valid && timeout < 100)
    {
        //printf("%s", line);
        timeout ++;
        switch (minmea_sentence_id(line, false)) {
            case MINMEA_SENTENCE_RMC: {
                struct minmea_sentence_rmc frame;
                gps->valid = true;
                if (minmea_parse_rmc(&frame, line)) {
                    //printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                    //        minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude), minmea_tofloat(&frame.speed)); 
                    gps->gps_coord.latitude = minmea_tocoord(&frame.latitude);
                    gps->gps_coord.longitude = minmea_tocoord(&frame.longitude);
                    gps->hour = frame.time.hours;
                    gps->minute = frame.time.minutes;
                    gps->second = frame.time.seconds;
                    gps->day = frame.date.day;
                    gps->month = frame.date.month;
                    gps->year = frame.date.year;
                    gps->true_track = minmea_tofloat(&frame.course);
                    gps->speed_kph = minmea_tofloat(&frame.speed) * 1.852;
                    gps->declin_mag = minmea_tofloat(&frame.variation);
                    gps->valid = frame.valid;
                }
                else {
                    //printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
                    gps->gps_coord.latitude = gps->gps_coord.longitude = gps->speed_kph = 0.0;
                    gps->valid = false;
                }
            } break;
            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, line)) {
                    //printf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
                    gps->fix_quality = frame.fix_quality;  // 0 = non valide, 1 = Fix GPS, 2 = Fix DGPS
                }
                else {
                    //printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
                    gps->fix_quality = 0;
                }
            } break;
             case MINMEA_SENTENCE_GST: {
                struct minmea_sentence_gst frame;
                 if (minmea_parse_gst(&frame, line)) {
                    //printf(INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                    //   minmea_tofloat(&frame.latitude_error_deviation), minmea_tofloat(&frame.longitude_error_deviation),
                    //   minmea_tofloat(&frame.altitude_error_deviation));
                }
                else {
                    //printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
                }
            } break;
            case MINMEA_SENTENCE_GSV: {
                struct minmea_sentence_gsv frame;
                if (minmea_parse_gsv(&frame, line)) {
                    /*printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                    printf(INDENT_SPACES "$xxGSV: satellites in view: %d\n", frame.total_sats);
                    for (int i = 0; i < 4; i++)
                        printf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n", frame.sats[i].nr,
                            frame.sats[i].elevation, frame.sats[i].azimuth, frame.sats[i].snr); */
                    gps->sat_in_view = frame.total_sats;
                }
                else {
                    //printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
                    gps->sat_in_view = 0;
                }
            } break;
            case MINMEA_INVALID: {
                //printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
            } break;
            default: {
                //printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
            } break;
        }
    }

    fclose(port_com);
    return 0;
}

/**
 * @brief		Change waypoint to next target
 * @param		Pointer to waypoint list and to GPS data 
 * @return		distance to next waypoint
 */
double goto_next_waypoint(Waypoint_list *wp_list, GPS_data *gps)
{
    //todo: PID goto wp

    double dist=0.0;
    dist = calculate_distance(wp_list->target_waypoint->wp_coordinate, &(gps->gps_coord));
    if(isnan(dist))
    {
         return 9999.9;
    }

    return dist;
}

/**
 * @brief		Print Navigation data to console, and save them in a file
 * @param		Pointer to waypoint list and to GPS data, distance to next waypoint
 * @return		0 during normal operation, -1 on error
 */
int print_and_log_nav(Waypoint_list *wpl, GPS_data *gps, double distance)
{
	static int print_header = 1;
	FILE *log_file;
    if ((log_file = fopen("log_bjva_nav.txt","a")) == NULL)
    {
        printf("\nError creating/opening log file\n");
        return -1;
    }
	if (print_header)
	{
		print_header=0;
		printf("\nNew Passage!");
		printf("\nWP ID,  Distance to WP, Bearing, GPS Qty, Fix Qual,   latitude,  longitude,       time,     date,  course,   speed\n");
		fprintf(log_file, "\nNew Passage!");
		fprintf(log_file, "\nWP ID,  Distance to WP, Bearing, GPS Qty, Fix Qual,   latitude,  longitude,       time,     date,  course,   speed\n");
	}
	printf("  %3d,        %8.3lf,   %5.1lf, ", wpl->target_waypoint->identification, distance, 
				calculate_bearing(&(gps->gps_coord), wpl->target_waypoint->wp_coordinate));
	fprintf(log_file, "  %3d,        %8.3lf,   %5.1lf, ", wpl->target_waypoint->identification, distance, 
				calculate_bearing(&(gps->gps_coord), wpl->target_waypoint->wp_coordinate));

    printf("     %02d,        %1d,", gps->sat_in_view, gps->fix_quality);
 	printf(" %10.6lf, %10.6lf, ", gps->gps_coord.latitude, gps->gps_coord.longitude);
	printf("  %02d:%02d:%02d, %02d/%02d/%2d, ", gps->hour, gps->minute, gps->second, gps->day, gps->month, gps->year);
	printf("  %5.1f,   %5.2f", gps->true_track, gps->speed_kph);
    printf("\n");

    fprintf(log_file, "     %02d,        %1d,", gps->sat_in_view, gps->fix_quality);
 	fprintf(log_file, " %10.6lf, %10.6lf, ", gps->gps_coord.latitude, gps->gps_coord.longitude);
	fprintf(log_file, "  %02d:%02d:%02d, %02d/%02d/%2d, ", gps->hour, gps->minute, gps->second, gps->day, gps->month, gps->year);
	fprintf(log_file, "  %5.1f,   %5.2f", gps->true_track, gps->speed_kph);
    fprintf(log_file, "\n");
	
	fclose(log_file);
	return 0;
}