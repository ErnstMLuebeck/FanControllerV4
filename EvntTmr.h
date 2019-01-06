#ifndef EVNTTMR_H
#define EVNTTMR_H

#include "Globals.h"


struct EventObj
{   volatile uint8_t id;
    volatile uint8_t d; // day: 0=every day, 1=monday,..
    volatile uint8_t h; // hour
    volatile uint8_t m; // minute
    volatile uint8_t s; // second
    volatile int new_state;
    char label[20];
    volatile EventObj* link; 
};

volatile EventObj* EvntHead;

void initEvents();
void registerEventObj(uint8_t id, uint8_t d, uint8_t h, uint8_t m, uint8_t s, int new_state, char* label);
void registerEventObj(uint8_t id, uint8_t d, uint8_t h, uint8_t m, uint8_t s, int new_state, char* label);
void printEvntList();
void clearEvntList();
int findEventObj(uint8_t d, uint8_t h, uint8_t m, uint8_t s);
int EventTimer();


void initEvents()
{
    head = NULL;

    /* only one event per second!
     *  
     */
    
    char label[10] = "WifiOff";
    registerEventObj(0, 0, 20, 04, 0, 0, label);

    char label1[10] = "WifiOn";
    registerEventObj(1, 0, 20, 04, 20, 1, label1);

    char label2[10] = "AlarmOn";
    registerEventObj(2, 0, 20, 12, 20, 1, label2);

    char label3[10] = "AlarmOff";
    registerEventObj(3, 0, 6, 0, 20, 1, label3);

    printEvntList();
}

void registerEventObj(uint8_t id, uint8_t d, uint8_t h, uint8_t m, uint8_t s, int new_state, char* label)
{
    noInterrupts();
    EventObj* addr = (EventObj*)malloc(sizeof(EventObj));
    addr->id = id;
    addr->d = d;
    addr->h = h;
    addr->m = m;
    addr->s = s;
    addr->new_state = new_state;

    strcpy(addr->label, label);
    
    // insert at beginning
    addr->link = EvntHead;  
    EvntHead = addr;
    noInterrupts();
}

int findEventObj(uint8_t d, uint8_t h, uint8_t m, uint8_t s)
{
    uint8_t obj_d, obj_h, obj_m, obj_s;
  
    // return id of first event object
    volatile EventObj* temp = EvntHead;
    while(temp != NULL)
    {
        obj_d = temp->d;
        obj_h = temp->h;
        obj_m = temp->m;
        obj_s = temp->s;

        if( (obj_d == d && obj_h == h) && (obj_m == m && obj_s == s))
            return((int)temp->id);

        temp = temp->link;
    } 
    return(-1); // no event object at this time 
}


void clearEvntList()
{
    noInterrupts();
    volatile EventObj* temp = EvntHead;
    volatile EventObj* del;
    while(temp != NULL)
    {
        del = temp->link;
        free((EventObj*)temp);
        temp = del;  
    }
    head = NULL;
    interrupts();
}

void printEvntList()
{
    // find end of list
    noInterrupts();
    volatile EventObj* temp = EvntHead;
    while(temp != NULL)
    {
        Serial.print(temp->id);
        Serial.print(": ");
        char str[20];
        sprintf(str, "%s", temp->label);
        Serial.print(str);
        Serial.print(", d=");
        Serial.print(temp->d);
        Serial.print(" h=");
        Serial.print(temp->h);
        Serial.print(" m=");
        Serial.print(temp->m);
        Serial.print(" s=");
        Serial.print(temp->s);
        Serial.println(" ");

        temp = temp->link;
    }
    Serial.println();
    interrupts();
}

int getEvntObjState(int id)
{
    // return id of first event object
    volatile EventObj* temp = EvntHead;
    while(temp != NULL)
    {     
        if(temp->id == id)
            return((int)temp->new_state);

        temp = temp->link;
    } 
    return(-1); // id not found
}


int EventTimer() 
{
    int EvntId = -1;
    EvntId = findEventObj(0, SC_SysHour, SC_SysMin, SC_SysSec);
    Serial.print(SC_SysHour);Serial.print(":");
    Serial.print(SC_SysMin);Serial.print(":");
    Serial.println(SC_SysSec);

    if(EvntId == -1)
    {   //Serial.println("No Event.");
        return(0);
    }

    int new_state = getEvntObjState(EvntId);

    switch(EvntId)
    {   case 0: Serial.print("Event0, "); 
            Serial.println(new_state); break;
        case 1: Serial.println("Event1, ");       
            Serial.println(new_state); break;
    }

}

#endif
