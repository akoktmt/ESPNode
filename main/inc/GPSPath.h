#ifndef GPS_P
#define GPS_P

typedef struct {
    float x;
    float y;
} Coordinate;
typedef enum{
    CAL_DIS_ANGLE=0,
    CONTROL_SPEED_ANGLE,
    COND_DIS,
    GET_KALMAN,
    STOP
} state;
typedef struct{
    float distance;
    float angle;
    float nextdistance;
}GPSpathData;
#define CRURENTX 
#define CRURENTY 
#define RANGE 
#define DISTANCETHRESH 1.0f
void get_current_location(Coordinate *loc);
void update_gps_data();
float calculate_distance(Coordinate a, Coordinate b);
float calculate_angle(float lat1, float lon1, float lat2, float lon2) ;
void GPSPathInit(void);
float haversine(float lat1, float lon1, float lat2, float lon2);
float toRadians(float degrees) ;
void calculate_orientation(Coordinate lot, Coordinate lotcurrent, Coordinate lotnext, float current_heading, float *initial_bearing, float *target_bearing, float *heading_offset, float *absolute_heading, float *heading_diff);
void initialize_orientation();
#endif /* GPS_P */