#ifndef DATALIST_H
#define DATALIST_H

#include <Arduino.h>

/**
 * @brief Item structure used in the DataList class 
 */
struct ListItem
{   /** item id */
    int id;
    /** data to store */
    int data;
    /** pointer to next item */
    ListItem* link;
    /** name of the item */
    char name[50];
};

/**
 * @brief Simple linked list of data structures: Id, data and name
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */ 
class DataList
{   
    public:
    DataList();
    void addItemHead(char* _name, int _data);
    char* getItemName(int _id);
    int getItemData(int _id);
    int getNumItems();
    void clearList();
    void printListConsole();
    
    private:
    ListItem* head;
    ListItem* tail;
    int numItems;
};

#endif /* DATALIST_H */
