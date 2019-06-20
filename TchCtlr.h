#ifndef TCHCTLR_H
#define TCHCTLR_H

#include "Globals.h"

// This is calibration data for the raw touch data to the screen coordinates
// Todo: calibrate scrren better
#define TS_MINX 3896 //150
#define TS_MINY 3763 //130
#define TS_MAXX 400 //3800
#define TS_MAXY 250 //4000

#define TFT_WIDTH 320
#define TFT_HEIGHT 240

#define COLOR_OUTL ILI9341_BLACK
#define COLOR_FILL ILI9341_BLUE
#define COLOR_TXT ILI9341_BLACK


int TC_IsTouched = false;
int TC_IsTouchedOld = false;

int TC_IsTouchedId = -1;
int TC_IsTouchedIdOld = -1;
int TC_IsTouchedType = -1;
int TC_IsTouchedTypeOld = -1;

int TC_WasTouched = false;
int tch_x = 0;
int tch_y = 0;

long int TiLastTouch = 0;
bool TchTimeout = 0;

XPT2046_Touchscreen ts(TCH_CS);

struct TouchObj
{   volatile uint8_t id;
    volatile int type;  // 0 = area, 1 = button, 2 = slider
    volatile uint16_t box[4];
    volatile TouchObj* link; 
    char label[20];
    volatile uint8_t state;
};

volatile TouchObj* head;

void initTouch();
void TouchController();
void registerTouchObj(uint8_t id, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void addButton(uint8_t id, uint16_t x, uint16_t y, uint16_t w, uint16_t h, char* label);
void addSlider(uint8_t id, uint16_t x, uint16_t y, uint16_t w, uint16_t h, char* label);
void ButtonUp(uint8_t id);
void ButtonDown(uint8_t id);
void SliderInactive(uint8_t id, float position);
void SwitchToggle(uint8_t id);
int findTouchObj();
void clearList();
void printList();


void initTouch()
{
    ts.begin();
    head = NULL;
    TiLastTouch = millis();
}

void registerTouchObj(uint8_t id, uint16_t x, uint16_t y, uint16_t w, uint16_t h, char* label, uint8_t type, uint8_t state)
{
    noInterrupts();
    TouchObj* addr = (TouchObj*)malloc(sizeof(TouchObj));
    addr->id = id;
    addr->box[0] = x;
    addr->box[1] = y;
    addr->box[2] = w;
    addr->box[3] = h;
    addr->type = type;
    addr->state = state;

    strcpy(addr->label, label);
    
    // insert at beginning
    addr->link = head;  
    head = addr;
    noInterrupts();
}

int findTouchObj(int x, int y)
{
    int x1, x2, y1, y2;
  
    // return id of first touched object
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {
        x1 = temp->box[0];
        x2 = temp->box[0]+temp->box[2];
        y1 = temp->box[1];
        y2 = temp->box[1]+temp->box[3];
        
      
        if( (x >= x1 && x <= x2) && (y >= y1 && y <= y2))
            return((int)temp->id);

        temp = temp->link;
    } 
    return(-1); // no object touched 
}

int getTouchObjType(int id)
{
    // return id of first touched object
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {     
        if(temp->id == id)
            return((int)temp->type);

        temp = temp->link;
    } 
    return(-1); // id not found
}

void clearList()
{
    noInterrupts();
    volatile TouchObj* temp = head;
    volatile TouchObj* del;
    while(temp != NULL)
    {
        del = temp->link;
        free((TouchObj*)temp);
        temp = del;  
    }
    head = NULL;
    interrupts();
}

void printList()
{
    // find end of list
    noInterrupts();
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {
        Serial.print(temp->id);
        Serial.print(": ");
        char str[20];
        sprintf(str, "%s", temp->label);
        Serial.print(str);
        Serial.print(", ");
        Serial.print(temp->box[0]);
        Serial.print(" y=");
        Serial.print(temp->box[1]);
        Serial.print(" w=");
        Serial.print(temp->box[2]);
        Serial.print(" h=");
        Serial.print(temp->box[3]);
        Serial.println(" ");

        temp = temp->link;
    }
    Serial.println();
    interrupts();
}

void addButton(uint8_t id, uint16_t x, uint16_t y, uint16_t w, uint16_t h, char* label)
{
    //uint16_t x = 40;
    //uint16_t y = 40;
    //uint16_t w = 120;
    //uint16_t h = 40;
    uint16_t fontsize = 18;

    registerTouchObj(id, x, y, w, h, label, 1, 0);
    ButtonUp(id);

    /*
    tft.drawRoundRect(x, y, w, h, 6, ILI9341_BLUE);
    
    tft.setTextColor(FGCOLOR, BGCOLOR);
    tft.setFont(Arial_18);

    tft.setCursor(x+w/2-tft.strPixelLen(label)/2, y+h/2-fontsize/2);
    tft.print(label);
    */
}

void addSlider(uint8_t id, uint16_t x, uint16_t y, uint16_t w, uint16_t h, char* label, uint16_t percent)
{
    //uint16_t x = 40;
    //uint16_t y = 40;
    //uint16_t w = 120;
    //uint16_t h = 40;
    uint16_t fontsize = 18;

    registerTouchObj(id, x, y, w, h, label, 2, percent);

    //x_slider = mapfloat(position, 0, 100, x, x+w);
    
    SliderInactive(id, percent);

}

void addSwitch(uint8_t id, uint16_t x, uint16_t y, uint16_t w, uint16_t h, char* label, uint16_t state)
{
    registerTouchObj(id, x, y, w, h, label, 3, !state);
    SwitchToggle(id);
}

void ButtonUp(uint8_t id)
{
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {
        if(temp->id == id)
            break;

        temp = temp->link;
    }

    uint16_t x = temp->box[0];
    uint16_t y = temp->box[1];
    uint16_t w = temp->box[2];
    uint16_t h = temp->box[3];
    uint16_t fontsize = 18;

    tft.fillRoundRect(x, y, w, h, 10, COLOR_FILL);
    tft.drawRoundRect(x, y, w, h, 10, COLOR_OUTL);
    
    tft.setTextColor(BGCOLOR, COLOR_FILL);
    tft.setFont(Arial_18);

    tft.setCursor(x+w/2-tft.strPixelLen((char*)temp->label)/2, y+h/2-fontsize/2);
    tft.print((char*)temp->label);
  
}

void ButtonDown(uint8_t id)
{
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {
        if(temp->id == id)
            break;

        temp = temp->link;
    }

    uint16_t x = temp->box[0];
    uint16_t y = temp->box[1];
    uint16_t w = temp->box[2];
    uint16_t h = temp->box[3];
    uint16_t fontsize = 18;

    tft.fillRoundRect(x, y, w, h, 10, BGCOLOR);
    tft.drawRoundRect(x, y, w, h, 10, COLOR_OUTL);
    //tft.drawRoundRect(x, y, w+1, h+1, 10, COLOR_OUTL);
    
    tft.setTextColor(COLOR_TXT, BGCOLOR);
    tft.setFont(Arial_18);

    tft.setCursor(x+w/2-tft.strPixelLen((char*)temp->label)/2, y+h/2-fontsize/2);
    tft.print((char*)temp->label);
  
}

void SliderInactive(uint8_t id, float position)
{
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {
        if(temp->id == id)
            break;

        temp = temp->link;
    }

    uint16_t x = temp->box[0];
    uint16_t y = temp->box[1];
    uint16_t w = temp->box[2];
    uint16_t h = temp->box[3];
    uint16_t fontsize = 18;
    uint16_t x_slider;

    tft.fillRect(x, y, w, h, COLOR_FILL);
    tft.drawRect(x, y, w, h, COLOR_OUTL);

    x_slider = mapfloat(position, 0, 100, x, x+w);
    tft.drawFastVLine(position-1, y, h, BGCOLOR);
    tft.drawFastVLine(position, y, h, BGCOLOR);
    tft.drawFastVLine(position+1, y, h, BGCOLOR);
    
    tft.setTextColor(FGCOLOR, COLOR_OUTL);
    tft.setFont(Arial_8);

    tft.setCursor(x, y-fontsize/2);
    tft.print((char*)temp->label);
}

void SliderActive(uint8_t id, float position)
{
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {
        if(temp->id == id)
            break;

        temp = temp->link;
    }

    uint16_t x = temp->box[0];
    uint16_t y = temp->box[1];
    uint16_t w = temp->box[2];
    uint16_t h = temp->box[3];
    uint16_t fontsize = 18;
    uint16_t x_slider;

    tft.fillRect(x, y, w, h, COLOR_FILL);
    tft.drawRect(x, y, w, h, COLOR_FILL);

    x_slider = mapfloat(position, 0, 100, x, x+w);
    tft.drawFastVLine(position-1, y, h, BGCOLOR);
    tft.drawFastVLine(position, y, h, BGCOLOR);
    tft.drawFastVLine(position+1, y, h, BGCOLOR);
    
    tft.setTextColor(FGCOLOR, COLOR_OUTL);
    tft.setFont(Arial_8);

    tft.setCursor(x, y-fontsize/2);
    tft.print((char*)temp->label);
}

void SwitchToggle(uint8_t id)
{
    volatile TouchObj* temp = head;
    while(temp != NULL)
    {
        if(temp->id == id)
            break;

        temp = temp->link;
    }

    uint16_t x = temp->box[0];
    uint16_t y = temp->box[1];
    uint16_t w = temp->box[2];
    uint16_t h = temp->box[3];
    uint16_t state1 = temp->state;

    tft.setFont(Arial_18);
    tft.setCursor(x+60, y+3);
    tft.print((char*)temp->label);

    if(state1) state1 = 0;
    else state1 = 1;

    temp->state = state1;

    if(state1)
    {   tft.fillRect(x, y, 50, 25, COLOR_FILL);
        tft.drawRect(x, y, 50, 25, COLOR_OUTL);
      
        tft.setTextColor(BGCOLOR, COLOR_FILL);
        tft.setFont(Arial_16);

        tft.setCursor(x+4, y+4);
        tft.print("ON");

        
    }
    else
    {   tft.fillRect(x, y, 50, 25, BGCOLOR);
        tft.drawRect(x, y, 50, 25, COLOR_OUTL);
      
        tft.setTextColor(FGCOLOR, BGCOLOR);
        tft.setFont(Arial_16);

        tft.setCursor(x+4, y+4);
        tft.print("OFF");
    }

    
}

void TouchController() 
{
    TC_IsTouched = ts.touched();
    //Serial.print(istouched);
    //Serial.println(TC_IsTouchedOld);    

    // finger touches the object (rising edge)
    if(TC_IsTouched  && !TC_IsTouchedOld)
    { 
        TiLastTouch = millis();
      
        TS_Point p = ts.getPoint();

        tch_x = map(p.x, TS_MINX, TS_MAXX, 0, TFT_WIDTH);
        tch_y = map(p.y, TS_MINY, TS_MAXY, 0, TFT_HEIGHT);

        Serial.print(tch_x);
        Serial.print(" ");
        Serial.println(tch_y);

        TC_IsTouchedId = findTouchObj(tch_x, tch_y);
        TC_IsTouchedType = getTouchObjType(TC_IsTouchedId);
        
        if(TC_IsTouchedId != -1)
        {   if(TC_IsTouchedType == 1)
                ButtonDown(TC_IsTouchedId);
            if(TC_IsTouchedType == 2)
                SliderActive(TC_IsTouchedId, tch_x);
            if(TC_IsTouchedType == 3);
                //SwitchToggle(istouched_id);
        }
    }

    // finger is still on the object
    /* Causes instability!!
     * 
     */
    if(TC_IsTouched && TC_IsTouchedOld)
    {   
        TS_Point p = ts.getPoint();

        tch_x = map(p.x, TS_MINX, TS_MAXX, 0, TFT_WIDTH);
        tch_y = map(p.y, TS_MINY, TS_MAXY, 0, TFT_HEIGHT);

        TC_IsTouchedId = findTouchObj(tch_x, tch_y);
        TC_IsTouchedType = getTouchObjType(TC_IsTouchedId);

        if(TC_IsTouchedType == 2)
        {
            SliderActive(TC_IsTouchedId, tch_x);
        }

        /* finger slided off of the object
        if((istouched_id == -1) && (TC_IsTouchedIdOld != -1))
        {   
            if(TC_IsTouchedTypeOld == 1)
                ButtonUp(TC_IsTouchedIdOld);
        }
        */
        TC_IsTouchedIdOld = TC_IsTouchedId;
        TC_IsTouchedTypeOld = TC_IsTouchedType;

    }

    // finger is released of the object
    if(!TC_IsTouched && TC_IsTouchedOld)
    {   TC_WasTouched = true;

        if(TC_IsTouchedType == 1)
            ButtonUp(TC_IsTouchedId);
        if(TC_IsTouchedType == 2)
            SliderInactive(TC_IsTouchedId, tch_x);
        if(TC_IsTouchedType == 3)
            SwitchToggle(TC_IsTouchedId);

        /* Todo:
         * Read values from sliders and toggle switches
         * Set last touch timer to zero -> screen saver
         */
        
    
    }
    
    TC_IsTouchedOld = TC_IsTouched;
}

#endif
