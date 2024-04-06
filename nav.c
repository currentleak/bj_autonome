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
        wp->target_radius = 0.01; // 10m
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
    wp->target_radius = 0.01; // 10m, destination waypoint has a closer target radius
    if(wp_list->waypoint_qty < 2)
    {
        printf("\nWaypoint list should contain at least 2 coordinates to make a passage\n");
        return NULL;
    }
    
    // Print the Waypoint List    
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
	
    wp_list->target_waypoint = wp_list->first_waypoint; // GPS coord Starting point
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

int get_gps_coordinate(GPS_data *gps)
{
    //data for simulation
/*    static double c[30]={46.3, -71.5, 46.5, -71.7, 46.68, -71.87, 46.6839, -71.8785, 46.68, -71.83, 46.685, -71.838, 46.68576, -71.83897, 
    46.668, -71.791, 46.668241, -71.791758, 46.666, -71.693, 46.666564, -71.693272, 46.673, -71.665, 46.673956, -71.665416, 46.697, -71.576, 46.69703, -71.57623};
    static int i=0;
    gps->gps_coord.latitude = c[i];
    gps->gps_coord.longitude = c[i+1];
    i=i+2;
    if(i==30) i=0;*/

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

            case MINMEA_SENTENCE_VTG: {
               struct minmea_sentence_vtg frame;
               if (minmea_parse_vtg(&frame, line)) {
                    //printf(INDENT_SPACES "$xxVTG: true track degrees = %f\n", minmea_tofloat(&frame.true_track_degrees));
                    //printf(INDENT_SPACES "        magnetic track degrees = %f\n", minmea_tofloat(&frame.magnetic_track_degrees));
                    //printf(INDENT_SPACES "        speed knots = %f\n", minmea_tofloat(&frame.speed_knots));
                    //printf(INDENT_SPACES "        speed kph = %f\n", minmea_tofloat(&frame.speed_kph)); 
               }
               else {
                    //printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
               }
            } break;

            case MINMEA_SENTENCE_ZDA: {
                struct minmea_sentence_zda frame;
                if (minmea_parse_zda(&frame, line)) {
                    //printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n", frame.time.hours, frame.time.minutes, 
                    //        frame.time.seconds, frame.date.day, frame.date.month, frame.date.year, frame.hour_offset, frame.minute_offset); 
                }
                else {
                    //printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
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

double goto_next_waypoint(Waypoint_list *wp_list, GPS_data *gps)
{
    //todo: PID goto wp

    double dist;
    dist = calculate_distance(wp_list->target_waypoint->wp_coordinate, gps->gps_coord);
    if(isnan(dist))
    {
         return 9999.9;
    }

    if(wp_list->target_waypoint == wp_list->first_waypoint)
    {
        printf("\nWaiting to be at starting line... \n");
    }

    printf("\rGPS Qty:%02d, Fix Qual:%1d, ", gps->sat_in_view, gps->fix_quality);
 	//printf("Distance todo= %8.3lf", calculate_distance(&gps->gps_coord, wp_list->target_waypoint->wp_coordinate));
/*	printf("   lat: %10.6lf, lon:%10.6lf", gps->gps_coord.latitude, gps->gps_coord.longitude);
	printf("   time: %02dh:%02dm:%02ds %02d/%02d/%2d", gps->hour, gps->minute, gps->second, gps->day, gps->month, gps->year);
	printf(" course: %5.1f, speed:%5.2f", gps.true_track, gps.speed_kph); */
	fflush(stdout);
    
	
/* 	printf("\nStart passage... time= %ld\n", time_passage);
	do
	{
		get_gps_coordinate(&gps);
		distance_to_target = goto_waypoint(&gps.gps_coord, waypoint_passage->target_waypoint);
		bearing_to_target = calculate_bearing(&gps.gps_coord, waypoint_passage->target_waypoint->wp_coordinate);
		printf("\rNext waypoint Id= %3d, Distance to next waypoint= %8.3lf, Bearing= %5.1lf", waypoint_passage->target_waypoint->identification, distance_to_target, bearing_to_target);
		fflush(stdout);
		if(!isnan(distance_to_target) && distance_to_target < waypoint_passage->target_waypoint->target_radius)
		{
			waypoint_passage->target_waypoint = waypoint_passage->target_waypoint->next_waypoint;
			printf("\n");
		}
		sleep(1);
	}
	while(waypoint_passage->target_waypoint != waypoint_passage->destination_waypoint);
 */

    printf("\nd=%lf",dist);
    return dist;
}