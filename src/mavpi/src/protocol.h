#ifndef _H_PRO_
#define _H_PRO_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DIV 10000
#define BufferLength 100
#define splitChar ','

//锁
enum planArmMode {
    NoArm,
    Arm,

    planArmMode_max
};

///飞行模式
enum planeFlyMode {
    Stable,
    Alt,
    Pos,
    OffBoard,

    planeFlyMode_max
};

//控制模式
enum CtrlMode {
    NoCtl, //未控制
    Pose, //点控制
    Raw, //原始控制

    CtrlMode_max
};

//上传报文
typedef struct
{
    int head;
    int index;
    int ctrlMode; //控制模式
    int pitch, roll; //原始模式
    int x, y, z, yaw; //定点模式，原始模式时z为油门，yaw偏航
    int div;
    int sumcheck;
    //char split = ',';
    int tail;
} upload_s;

typedef struct
{
    int head;
    int index;
    int armed; //锁
    int flyMode; //飞行模式
    int x, y, z, yaw; //位置信息
    int x_set, y_set, z_set, yaw_set; //位置设置
    int pitch; //当前值
    int roll;
    int pitch_set, roll_set; //设定值
    int dis; //地面距离
    int div; //倍数
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
