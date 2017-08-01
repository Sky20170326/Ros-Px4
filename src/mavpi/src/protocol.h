#ifndef _H_PRO_
#define _H_PRO_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DIV 100
#define BufferLength 100
#define splitChar ','

enum planArmMode {
    NoArm,
    Arm
};

enum planeFlyMode {
    Stable,
    Alt,
    Pos,
    OffBoard
};

enum CtrlMode {
    Pose,
    Raw
};

typedef struct
{
    int head;
    int index;
    int ctrlMode;
    int pitch, roll;
    int x, y, z, yaw;
    int div;
    int sumcheck;
    //char split = ',';
    int tail;
} upload_s;

typedef struct
{
    int head;
    int index;
    int armed;
    int flyMode;
    int x, y, z, yaw;
    int x_set, y_set, z_set, yaw_set;
    int pitch;
    int roll;
    int pitch_set, roll_set;
    int dis;
    int div;
    int sumcheck;
    //char split = ',';
    int tail;
} download_s;

char* packUpload(upload_s* pack);
char* packDownload(download_s* pack);

upload_s makeUpPack(enum CtrlMode ctrlMode, float pitch, float roll,
    float x, float y, float z, float yaw);
download_s makeDownPack(enum planArmMode arm, enum planeFlyMode flyMode,
    float x, float y, float z, float yaw,
    float x_set, float y_set, float z_set, float yaw_set,
    float pitch, float roll,
    float pitch_set, float roll_set,
    float dis);

bool unpackUp(char* str, upload_s* p);
bool unpackDown(char* str, download_s* p);

bool readFilter(char* c);
bool CheckString(char* c);

#ifdef __cplusplus
}
#endif

#endif
