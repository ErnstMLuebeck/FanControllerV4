#ifndef LIST_H
#define LIST_H

#include <Arduino.h>

struct ListItem
{   int id;
    ListItem* link;  // link to next item
    char name[50];   // name of the itme
    float value;     // value of the item
};

class List
{   public:
    List();
    void addItemHead(char* _name, float _value);
    char* getItemName(int _id);
    float getItemValue(int _id);
    int getItemIdByName(char* _name);
    int getNumItems();
    void clearList();
    void printListConsole();
    
    private:
    ListItem* head;
    ListItem* tail;
    int numItems;
    
};

#endif
