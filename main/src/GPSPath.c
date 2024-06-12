// File Server
#include "app_internal.h"
// Freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//
#include <math.h>
#include "GPSPath.h"
#include "EKF_exe.h"
extern  EKF EKFexe;
extern bool Headingtrue;
uint8_t firstinit=1; 
extern uint8_t GPSrec;
Coordinate route[] = {
{
  10.869327, 106.802869
},
{10.869329, 106.802825

},
{10.869326, 106.802912

},
{10.869325, 106.802970

},
{
  10.869324, 106.803024

},
{
  10.869323, 106.803087

},
{10.869323, 106.803087

},
{
  10.869319, 106.803191
},
{
10.869316, 106.803225
},
{
  10.869317, 106.803289
},
{
  10.869315, 106.803349
},
{
  10.869313, 106.803407
},
{
  10.869311, 106.803475
},
{
  10.869311, 106.803538
},
{
  10.869309, 106.803601
},
{
  10.869305, 106.803664
},
{
  10.869296, 106.803703
}
};
Coordinate lot;
Coordinate lotnext;
Coordinate lotcurrent;
state GPSstate ; 
GPSpathData DataPath;
extern uint8_t pid_speed;
extern bool Headingtrue;
const int total_points = sizeof(route) / sizeof(route[0]);
int current_index = 0;
bool firstDistance =true,Cal_distance=false,Lastpoint=false,Kalman=false,Cal_angle=false,Initorientation=false;
bool GPSpathOK=false;
float heading;
uint8_t stee_speed;
// 
float global_heading_offset;
float current_heading;
float initial_bearing=0;
float target_bearing;
float absolute_heading;
float heading_diff;
float heading_offset;
float distance;
uint8_t time_speed=0;
bool RunCalib=false,InitSignal=false;
//10.869853, 106.802338
void initialize_orientation(){
    // current_heading = EKFexe.FirHea;
    lot = route[current_index]; // start lot
   // printf("Lot x %f Lot y %f\n", lot.x,lot.y);
    lotcurrent.x = EKFexe.FirPx;    // lot current10.869333, 106.802575
    lotcurrent.y = EKFexe.FirPy;    // lot current
  //  printf("lotcurrent x %f lotcurrent y %f\n", lotcurrent.x,lotcurrent.y);
    current_index++ ; // move to next lot
   // printf("current_index %d\n",current_index);
    lotnext =route[current_index]; // Next Current target lot
  //  printf("lotnext x %f lotnext y %f\n", lotnext.x,lotnext.y);
    //--------------------------------------- 
    current_heading= EKFexe.FirHea; 
    calculate_orientation(lot, lotcurrent, lotnext, current_heading, &initial_bearing, &target_bearing, &heading_offset, &absolute_heading, &heading_diff);
   // printf("heading_diff %f\n ",heading_diff);
    InitSignal=true;
    //----------------------------------------
    // distance = haversine_distance(new_lat, new_lon, goal_lat, goal_lon);
    // printf("Distance to goal: %f meters\n", distance);
}
void controlSpeedTask() {
    // Set tốc độ ban đầu
    for(;;){
    if(GPSrec==1){
    pid_speed = 5;
    heading_diff=0;
   // printf("Tốc độ được set là: %d\n", pid_speed);

    // Chờ trong 5 giây
    vTaskDelay(pdMS_TO_TICKS(8000));

   // Giảm tốc độ dần dần
    // for (int i = 0; i < 10; i++) {
      //  vTaskDelay(pdMS_TO_TICKS(500)); // Giảm tốc độ mỗi 0.5 giây
       // pid_speed -= 2; // Giảm 2 đơn vị mỗi lần
       // printf("Tốc độ hiện tại là: %d\n", pid_speed);
    // }
    // Đảm bảo tốc độ cuối cùng là 0
    pid_speed = 1;
    // Signal Calib turn on
    RunCalib =true;
    vTaskDelete(NULL);
    }
     vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
float distance2lot;
void GPSpath(){
  for(;;){
    if(RunCalib==true){
        if(GPSrec==1 && Initorientation!=true){
            if(Headingtrue==true){
                if(InitSignal!=true){
                        initialize_orientation();
                        vTaskDelay(pdMS_TO_TICKS(5000));
                        pid_speed=5;
                        GPSstate=CAL_DIS_ANGLE; 
                        Initorientation=true;
                     //   printf("done init\n");
            }
        }
        }
     if(Initorientation==true){
        if(InitSignal==true){
                switch (GPSstate) {
                    case CONTROL_SPEED_ANGLE:
                    if(Headingtrue==true){
                            pid_speed=5;
                            current_heading =EKFexe.FirHea;
                            calculate_orientation(lot, lotcurrent, lotnext, current_heading, &initial_bearing, &target_bearing, &heading_offset, &absolute_heading, &heading_diff);
                            GPSstate=CAL_DIS_ANGLE;
                       //    printf("Speed_angle: %f\n",heading_diff);
                    }
                    else
                    {
                     //   printf("Stop state speed angle\n");
                        pid_speed=1;
                        GPSstate = CONTROL_SPEED_ANGLE;
                    }
                            break;
                    case CAL_DIS_ANGLE:
                            distance2lot=haversine(lotcurrent.x,lotcurrent.y,lotnext.x,lotnext.y);
                         //   printf("distance2lot: %f\n", distance2lot);
                            GPSstate=COND_DIS;
                            break;
                    case COND_DIS:
                            if(distance2lot<=DISTANCETHRESH){
                                if(current_index==total_points-1){
                                  //  printf("condis_stop\n");
                                    GPSstate=STOP;
                                }
                                else{
                                  //  printf("Condis_getkalman\n");
                                    current_index++;
                                    lotnext=route[current_index];
                                    GPSstate=GET_KALMAN;
                                }
                            }
                            else{
                               // printf("condis_else_kalman\n");
                                GPSstate=GET_KALMAN;
                            } 
                                break;
                    case GET_KALMAN:
                    if(Kalman==true){
                            lotcurrent.x=EKFexe.FirPx;
                            lotcurrent.y=EKFexe.FirPy;
                            GPSstate=CONTROL_SPEED_ANGLE;
                    }
                    else {
                            lotcurrent.x=EKFexe.NexPx;
                            lotcurrent.y=EKFexe.NexPy;
                            GPSstate=CONTROL_SPEED_ANGLE;
                    }
                          //  printf("Kalman_get\n");
                            //printf("Getkalmanstate\n");
                                break;
                    case STOP:
                       // printf("casestop\n");
                            pid_speed=1;
                            vTaskDelete(NULL);
                            break;
                            }
                        }
        
     }
    }
    vTaskDelay(50/portTICK_PERIOD_MS);
}

}
void GPSPathInit(){
    xTaskCreate(controlSpeedTask, "ControlSpeedTask", 2048, NULL, 13, NULL);
    xTaskCreate(GPSpath, "GPSpath", 1024 * 5, NULL, 14, NULL);
}

float toRadians(float degrees) {
    return degrees * PI / 180.0;
}
float haversine(float lat1, float lon1, float lat2, float lon2) {
    // Convert latitude and longitude from degrees to radians
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);
    
    // Haversine formula
    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;
    float a = sin(dlat / 2) * sin(dlat / 2) + 
               cos(lat1) * cos(lat2) * 
               sin(dlon / 2) * sin(dlon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    // Distance in meters
    float distance = EARTH_RADIUS * c; 
    
    return distance;
}
float calculate_angle(float lat1, float lon1, float lat2, float lon2) {
    float dLon = toRadians(lon2 - lon1);
    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);

    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float bearing = atan2(y, x);

    return fmod((bearing * 180 / PI + 360), 360); 
}
void calculate_orientation(Coordinate lot, Coordinate lotcurrent, Coordinate lotnext, float current_heading, float *initial_bearing, float *target_bearing, float *heading_offset, float *absolute_heading, float *heading_diff) {
    // Calculate initial bearing
    *initial_bearing = calculate_angle(lot.x, lot.y, lotcurrent.x, lotcurrent.y);
  //  printf("Initial bearing: %f\n", *initial_bearing);

    // Calculate target bearing
    *target_bearing = calculate_angle(lotcurrent.x, lotcurrent.y, lotnext.x, lotnext.y);
 //   printf("Target bearing: %f\n", *target_bearing);

    // Calculate heading offset
    *heading_offset = *target_bearing - *initial_bearing;
  //  printf("Global heading offset: %f\n", *heading_offset);

    // Calculate absolute heading
    *absolute_heading = fmod(current_heading + *heading_offset, 360.0);
 //   printf("Absolute heading: %f\n", *absolute_heading);

    // Calculate heading difference
    *heading_diff = fmod(*target_bearing - *absolute_heading + 540, 360) - 180;
  //  printf("Heading difference: %f\n", *heading_diff);
}