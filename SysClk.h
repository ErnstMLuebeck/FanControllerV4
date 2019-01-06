#ifndef SYSCLK_H
#define SYSCLK_H

/* TODO:
 * DST not working arount midnight (-Dst)
 *  
 */

#include "Globals.h"
#include <Timezone.h> 

// System Clock
int SC_SysHour, SC_SysMin, SC_SysSec; 
int SC_SysDay, SC_SysMon, SC_SysYear;
int SC_Dst = 1; 

//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);

TimeChangeRule *tcr;        //pointer to the time change rule, use to get the TZ abbrev
time_t utc;

void setSystemTime();
void getSystemTime(int* h, int* mi, int* sec);
void getSystemDate(int* d, int* mon, int* y);
void getDst(int* dst);
time_t compileTime(void);
void printTime(time_t t, char *tz, char *loc);
void sPrintI00(int val);
void sPrintDigits(int val);

void setSystemTime()
{
  setTime(compileTime());
  //setTime(15,55,0,15,8,2017);
}

void getSystemTime(int* h, int* mi, int* sec)
{
    *h = hour();
    *mi = minute();
    *sec = second();
}

void getSystemDate(int* d, int* mon, int* y)
{
    *d = day();
    *mon = month();
    *y = year();  
}

void getDst(int* dst)
{
    //todo: automatically determine DST
    *dst = 1;
}

//Function to return the compile date and time as a time_t value
time_t compileTime(void)
{
    const time_t FUDGE(25);        //fudge factor to allow for compile time (seconds, YMMV)
    char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char chMon[4], *m;
    int d, y;
    tmElements_t tm;
    time_t t;

    strncpy(chMon, compDate, 3);
    chMon[3] = '\0';
    m = strstr(months, chMon);
    tm.Month = ((m - months) / 3 + 1);

    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime) - SC_Dst;   // ATTENTION: DST IS HARDCODED HERE!!
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);
    t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
}

#endif
