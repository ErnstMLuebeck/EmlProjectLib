#include "DataList.h"

/** 
 * Construct list object with null pointers
 */
DataList::DataList()
{
    head = NULL;
    tail = NULL;
    numItems = 0;
}

/** 
 * @brief Add a data item to the head of the list
 * 
 * @param _name pointer to name string (maximum 50 characters!)
 * @param _data data to be stored
 */
void DataList::addItemHead(char* _name, int _data)
{
    int id = numItems;
    numItems++;
    
    noInterrupts();
    
    ListItem* addr = (ListItem*)malloc(sizeof(ListItem));
    addr->id = id;
    addr->data = _data;
    
    strcpy(addr->name, _name);
    
    /* insert item at beginning of the list */
    addr->link = head;
    head = addr;
    
    interrupts();
}

/**
 * @brief Get number of items in the list
 * 
 * @return number of items stored in the list
 */
int DataList::getNumItems()
{
    return(numItems);
}

/**
 * @brief Delete complete list and free memory
 * 
 */
void DataList::clearList()
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

/**
 * @brief Get name of item based on ID
 * 
 * @param _id Item ID
 * @return pointer to name string or NULL if item does not exist
 */
char* DataList::getItemName(int _id)
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

/**
 * @brief Get data value of item based on ID
 * 
 * @param _id Item ID
 * @return Data value or -1 if item does not exist
 */
int DataList::getItemData(int _id)
{
    noInterrupts();
    ListItem* temp = head;
    while(temp != NULL)
    {
        if(temp->id == _id)
        {   interrupts();
            return(temp->data);
        }
        temp = temp->link;
    }
    interrupts();
    return(-1);
}

/**
 * @brief Print full list to console
 * 
 */
void DataList::printListConsole()
{
    // find end of list
    noInterrupts();
    ListItem* temp = head;
    while(temp != NULL)
    {
        Serial.print(temp->id);
        Serial.print(": ");
        char str[30];
        sprintf(str, "%s", temp->name);
        Serial.print(str);
        Serial.print(", data=");
        Serial.print(temp->data);
        
        Serial.println("; ");
        
        temp = temp->link;
    }
    Serial.println();
    interrupts();
}
