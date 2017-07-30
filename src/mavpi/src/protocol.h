#ifndef _H_PRO_
#define _H_PRO_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <string.h>
#include <stdbool.h>


#define DIV 100
#define splitChar ','

typedef struct
{
        int head;
        int index;
        int x,y,z,yaw;
        int div;
        int sumcheck;
        //char split = ',';
        int tail;
} upload_s;

typedef struct
{
        int head;
        int index;
        int x,y,z,yaw;
        int x_set,y_set,z_set,yaw_set;
        int div;
        int sumcheck;
        //char split = ',';
        int tail;
} download_s;

char * packUpload(upload_s * pack);
char * packDownload(download_s * pack);

upload_s makeUpPack(float x,float y,float z,float yaw);
download_s makeDownPack(float x,float y,float z,float yaw,
                        float x_set,float y_set,float z_set,float yaw_set);

bool unpackUp (char * str,upload_s * p);
bool unpackDown (char * str,download_s * p);

bool readFilter(char * c);
bool CheckString(char *c);

#ifdef __cplusplus
}
#endif

#endif
