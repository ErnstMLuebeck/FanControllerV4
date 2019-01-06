
#include "List.h"

List::List()
{
    head = NULL;
    tail = NULL;
    numItems = 0;
}

void List::addItemHead(char* _name, float _value)
{
    int id = numItems;
    numItems++;
    
    noInterrupts();
    
    ListItem* addr = (ListItem*)malloc(sizeof(ListItem));
    addr->id = id;
    addr->value = _value;
    
    strcpy(addr->name, _name);
    
    // insert at beginning
    addr->link = head;
    head = addr;
    
    interrupts();
}

int List::getNumItems()
{
    return(numItems);
}

void List::clearList()
{
    noInterrupts();
    ListItem* temp = head;
    ListItem* del;
    while(temp != NULL)
    {
        del = temp->link;
        free((ListItem*)temp);
        temp = del;
    }
    head = NULL;
    interrupts();
}

char* List::getItemName(int _id)
{
    noInterrupts();
    ListItem* temp = head;
    while(temp != NULL)
    {   //Serial.println(temp->id);
        if(temp->id == _id)
        {   //char str[30];
            //sprintf(str, "%s", temp->name);
            //Serial.println(str);
            interrupts();
            return(temp->name);
        }
        temp = temp->link;
    }
    interrupts();
    return(NULL);
}

int List::getItemIdByName(char* _name)
{
    noInterrupts();
    ListItem* temp = head;
    while(temp != NULL)
    {           
        if(!strcmp(temp->name, _name))
        {   
            interrupts();
            return(temp->id);
        }
        temp = temp->link;
    }
    interrupts();
    return(-1);
}

float List::getItemValue(int _id)
{
    noInterrupts();
    ListItem* temp = head;
    while(temp != NULL)
    {
        if(temp->id == _id)
        {   interrupts();
            return(temp->value);
        }
        temp = temp->link;
    }
    interrupts();
    return(-1);
}

void List::printListConsole()
{
    // find end of list
    noInterrupts();
    ListItem* temp = head;
    while(temp != NULL)
    {
        Serial.print(temp->id);
        Serial.print(": ");
        char str[32];
        sprintf(str, "%s", temp->name);
        Serial.print(str);
        Serial.print(" = ");
        Serial.print(temp->value,3);
        
        Serial.println("; ");
        
        temp = temp->link;
    }
    Serial.println();
    interrupts();
}
