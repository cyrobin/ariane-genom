#ifndef ariane_STRUCT_H
#define ariane_STRUCT_H

#define ariane_REGION_FILE_PATH "/tmp/map.tif"
#define ariane_ROBOT_FILE_PATH "/tmp/robot.json"
#define ariane_POM_POSTER_NAME "pomPos"
#define ariane_DTM_POSTER_NAME "dtm"
#define ariane_MAX_STEP_LENGTH "10.0"
#define ariane_CURB_TOLERANCE "0.2"
#define ariane_DEFAULT_LOGDIR "/tmp"
#define ariane_MAX_LENGTH 1024

typedef struct arianeInternalParams {
    double max_step_length;
    double curb_tolerance;
} arianeInternalParams;

typedef struct arianeInitParams {
	char f_region[ariane_MAX_LENGTH];
	char f_robot[ariane_MAX_LENGTH];
	char dtmPosterName[ariane_MAX_LENGTH];
	char pomPosterName[ariane_MAX_LENGTH];
    double max_step_length;
    double curb_tolerance;
	char logDir[ariane_MAX_LENGTH];
} arianeInitParams;

#endif //ariane_STRUCT_H
