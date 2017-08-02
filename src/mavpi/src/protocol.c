
#include "protocol.h"

char stru[BufferLength];

char* packUpload(upload_s* pack)
{
    sprintf(stru, "%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c",
        (char)pack->head,
        pack->index,
        pack->ctrlMode,
        pack->pitch,
        pack->roll,
        pack->x,
        pack->y,
        pack->z,
        pack->yaw,
        pack->div,
        pack->sumcheck,
        (char)pack->tail);
    return stru;
}

char  strd[BufferLength];
char* packDownload(download_s* pack)
{
    sprintf(strd, "%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c",
        (char)pack->head,
        pack->index,
        pack->armed,
        pack->flyMode,
        pack->x,
        pack->y,
        pack->z,
        pack->yaw,
        pack->x_set,
        pack->y_set,
        pack->z_set,
        pack->yaw_set,
        pack->pitch,
        pack->roll,
        pack->pitch_set,
        pack->roll_set,
        pack->dis,
        pack->div,
        pack->sumcheck,
        (char)pack->tail);
    return strd;
}

int sumcheck(int* addr, int dataLength)
{
    int num = 0;
    for (int i = 0; i < dataLength - 2; i++) {
        num += *(addr + i);
    }
    return abs(num % 100);
}

int upIndex = 0;

upload_s makeUpPack(enum CtrlMode ctrlMode, float pitch, float roll,
    float x, float y, float z, float yaw)
{
    upload_s p;
    p.head  = (int)'r';
    p.index = upIndex++;
    // p.index = 7;
    if (upIndex >= 100)
        upIndex = 0;
    p.ctrlMode  = (int)ctrlMode;
    p.pitch     = (int)(pitch * DIV);
    p.roll      = (int)(roll * DIV);
    p.x         = (int)(x * DIV);
    p.y         = (int)(y * DIV);
    p.z         = (int)(z * DIV);
    p.yaw       = (int)(yaw * DIV);
    p.div       = DIV;
    p.sumcheck  = 0;
    p.tail      = (int)'\n';
    //printf("check!");
    p.sumcheck = sumcheck((int*)&p, sizeof(upload_s) / sizeof(int));
    return p;
}

int        downIndex = 0;
download_s makeDownPack(enum planArmMode arm, enum planeFlyMode flyMode,
    float x, float y, float z, float yaw,
    float x_set, float y_set, float z_set, float yaw_set,
    float pitch, float roll,
    float pitch_set, float roll_set,
    float dis)
{
    download_s p;
    p.head  = (int)'s';
    p.index = downIndex++;
    if (downIndex >= 100)
        downIndex = 0;
    p.armed       = (int)arm;
    p.flyMode     = (int)flyMode;
    p.x           = (int)(x * DIV);
    p.y           = (int)(y * DIV);
    p.z           = (int)(z * DIV);
    p.yaw         = (int)(yaw * DIV);
    p.x_set       = (int)(x_set * DIV);
    p.y_set       = (int)(y_set * DIV);
    p.z_set       = (int)(z_set * DIV);
    p.yaw_set     = (int)(yaw_set * DIV);
    p.pitch       = (int)(pitch * DIV);
    p.roll        = (int)(roll * DIV);
    p.pitch_set   = (int)(pitch_set * DIV);
    p.roll_set    = (int)(roll_set * DIV);
    p.dis         = (int)(dis * DIV);
    p.div         = DIV;
    p.sumcheck    = 0;
    p.tail        = (int)'\n';
    p.sumcheck    = sumcheck((int*)&p, sizeof(download_s) / sizeof(int));
    return p;
}

int unpack(char* str, int* p, int maxLength)
{
    //printf("%s \n",str);
    //int* _p = p;

    //int  packLength      = 0;
    char splitCharPtr[2] = { splitChar, '\0' };
    char str_temp[BufferLength];
    strcpy(str_temp, str);
    char* s;
    s    = strtok(str_temp, splitCharPtr);
    *p++ = *s;
    // printf("%s \n", s);

    int num = 1;
    while (s = strtok(NULL, splitCharPtr)) {
        //printf("%s \n",s);
        if (*s >= '0' && *s <= '9') {
            *p++ = atoi(s);
        } else {
            *p++ = *s;
            break;
        }

        if (++num >= maxLength)
            break;
    }

    return ++num;
}

bool unpackUp(char* str, upload_s* p)
{
    // printf("check%d!\n", sizeof(upload_s) / sizeof(int));
    // printf("check%d!\n", unpack(str, p, sizeof(upload_s) / sizeof(int)));

    int* _p = (int*)p;
    //length check
    int num = unpack(str, _p, sizeof(upload_s) / sizeof(int));
    if (sizeof(upload_s) / sizeof(int) == num)
        if (p->head == 'r'
            && p->tail == '\n'
            && p->sumcheck == sumcheck((int*)p, sizeof(upload_s) / sizeof(int)))
            return true;

    return false;
}

bool unpackDown(char* str, download_s* p)
{
    int* _p  = (int*)p;
    int  num = unpack(str, _p, sizeof(download_s) / sizeof(int));
    //printf("\nthis:%d %d\n",num,sizeof(download_s) / sizeof(int));
    if (sizeof(download_s) / sizeof(int) == num)
        if (p->head == 's'
            && p->tail == '\n'
            && p->sumcheck == sumcheck((int*)p, sizeof(download_s) / sizeof(int)))
            return true;

    return false;
}

bool readFilter(char* c)
{
    return (
        *c == 'r' || *c == 's' //head
        || *c == '\n' //tail
        || (*c >= '0' && *c <= '9') //number
        || (*c == ',') //split
        );
}

bool CheckString(char* c)
{
    bool r = (*c != '\0');
    while (*c != '\0') {
        r = r && readFilter(c++);
    }
    return r;
}
