
#include "protocol.h"

char stru[20];

char * packUpload(upload_s *pack)
{
        sprintf(stru,"%c,%d,%d,%d,%d,%d,%d,%d,%c",
                (char)pack->head,
                pack->index,
                pack->x,
                pack->y,
                pack->z,
                pack->yaw,
                pack->div,
                pack->sumcheck,
                (char)pack->tail
                );
        return stru;
}

char strd[20];
char * packDownload(download_s * pack)
{
        sprintf(strd,"%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c",
                (char)pack->head,
                pack->index,
                pack->x,
                pack->y,
                pack->z,
                pack->yaw,
                pack->x_set,
                pack->y_set,
                pack->z_set,
                pack->yaw_set,
                pack->div,
                pack->sumcheck,
                (char)pack->tail
                );
        return strd;
}

int sumcheck(int * addr,int sumCheck)
{
        int num = -sumCheck;
        addr++;
        while(*addr != '\n')
        {
                num += *addr++;
        }
        return num % 100;
}

int upIndex = 0;

upload_s makeUpPack(float x,float y,float z,float yaw)
{
        upload_s p;
        p.head = (int)'r';
        p.index = upIndex++;
        p.x = (int)(x * DIV);
        p.y = (int)(y * DIV);
        p.z = (int)(z * DIV);
        p.yaw = (int)(yaw * DIV);
        p.div = DIV;
        p.sumcheck = 0;
        p.tail = (int)'\n';
        //printf("check!");
        p.sumcheck = sumcheck ((int *) &p,p.sumcheck);
        return p;
}

int downIndex = 0;
download_s makeDownPack(float x,float y,float z,float yaw,
                        float x_set,float y_set,float z_set,float yaw_set)
{
        download_s p;
        p.head = (int)'s';
        p.index = downIndex++;
        p.x = (int)(x * DIV);
        p.y = (int)(y * DIV);
        p.z = (int)(z * DIV);
        p.yaw = (int)(yaw * DIV);
        p.x_set = (int)(x_set * DIV);
        p.y_set = (int)(y_set * DIV);
        p.z_set = (int)(z_set * DIV);
        p.yaw_set = (int)(yaw_set * DIV);
        p.div = DIV;
        p.sumcheck = 0;
        p.tail = (int)'\n';
        p.sumcheck = sumcheck ((int *) &p,p.sumcheck);
        return p;
}

int unpack(char * str,int * p,int maxLength)
{
        //printf("%s \n",str);
        int * _p = p;

        int packLength = 0;
        char splitCharPtr [2] = {splitChar,'\0'};

        char *s;
        s = strtok(str,splitCharPtr);
        *p++ = *s;
        //printf("%s \n",s);

        int num = 1;
        while(s = strtok(NULL,splitCharPtr))
        {
                //printf("%s \n",s);
                if(*s >= '0' && *s <= '9')
                {
                        *p++ = atoi(s);
                }
                else
                {
                        *p++ = *s;
                        break;
                }

                if(++num >= maxLength)
                        break;
        }


        return ++num;
}


bool unpackUp (char * str,upload_s * p)
{
        //printf("check%d!\n",sizeof(upload_s) / sizeof(int));
        //printf("check%d!\n",unpack(str,p,sizeof(upload_s) / sizeof(int)));

        int * _p = (int *) p;
        //length check
        int num = unpack(str,_p,sizeof(upload_s) / sizeof(int));
        if(sizeof(upload_s) / sizeof(int) == num
           )
                if(p->head == 'r'
                   && p->tail == '\n'
                   && p->sumcheck == sumcheck((int *)p,p->sumcheck)
                   )
                        return true;
/*
        printf("data:%d,%d %d %d %d,%d,%d,%d %d %d %d %d\n",sizeof(upload_s) / sizeof(int),num,
               p->head,
               p->index,
               p->x,p->y,p->z,p->yaw,
               p->div,
               p->sumcheck,
               p->tail,
               sumcheck((int *)p,p->sumcheck));
 */
        return false;
}



bool unpackDown (char * str,download_s * p)
{
        int * _p = (int *)p;
        if(sizeof(download_s) / sizeof(int) ==
           unpack(str,p,sizeof(download_s) / sizeof(int)))
                if(p->head == 's'
                   && p->tail == '\n'
                   && p->sumcheck == sumcheck((int *)p,p->sumcheck))
                        return true;
        return false;
}


bool readFilter(char * c)
{
        return (
                       *c == 'r' || *c == 's' //head
                       || *c == '\n' //tail
                       || (*c >= '0' && *c <= '9') //number
                       );
}

bool CheckString(char *c)
{
        bool r = (*c != '\0');
        while(*c != '\0')
        {
                r = r && readFilter(c++);
        }
        return r;
}
