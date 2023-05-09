#include <utility>
#include <vector>
#include <iostream>
#include <queue>
#include <iomanip>
#include <unordered_set>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <unordered_map>


#include "Object.h"


#ifndef ROBOTNAVIGATION_MAP_H
#define ROBOTNAVIGATION_MAP_H


// A MapItem is an item in the Map, consisting of a distance and an object
struct MapItem {
public:
    double dist;              // distance from the object
    Object::Ptr obj;          // object pointer


public:
    // Constructor to initialize MapItem with distance and object
    explicit MapItem(double d, Object::Ptr o) : dist(d), obj(std::move(o)) {}
};


// A MapComp is a comparison object used by std::priority_queue to sort MapItems by distance
struct MapComp {
public:
    // Operator to compare two MapItems by distance
    bool operator()(const MapItem &m1, const MapItem &m2) const {
        return m1.dist > m2.dist;
    }
};


// The Map class represents a map of objects with obstacles
class Map {
public:
    const int rows;     // number of rows in the map
    const int cols;     // number of columns in the map
    using Ptr = std::shared_ptr<Map>;    // shared pointer to Map


private:
    std::vector<double> vert_dist_;    // vertical distances from edge
    std::vector<double> hor_dist_;     // horizontal distances edge
    std::vector<std::priority_queue<MapItem, std::vector<MapItem>, MapComp>> heat_map_;    // heat map of MapItems sorted by distance
    std::unordered_set<Object::Ptr> obstacles;    // set of obstacle object pointers


public:
    // Constructor to create a Map with specified number of rows and columns
    Map(int r, int c);


    // Static method to create a Map shared pointer with specified number of rows and columns
    static Map::Ptr createMap(int r, int c);


    // Method to get the number of objects in the Map
    int numObjects();


    // Method to add an object to the Map
    bool addObject(const Object::Ptr &object);


    // Method to add an object to the Map with specified x and y coordinates and radius
    Object::Ptr addObject(int x, int y, double r);


    // Method to remove an object from the Map
    bool removeObject(const Object::Ptr &object);


    // Method to remove an object from the Map with specified x and y coordinates and radius
    Object::Ptr removeObject(int x, int y, double r);


    // Method to clear the Map of all objects and obstacles
    void clearMap();


    // Method to get a vector of obstacle object pointers in the Map
    [[nodiscard]] std::vector<Object::Ptr> getObstacles() const;


    // Method to get the value at a specified coordinate in the Map
    double valAt(const Coord &c);


    // Method to get the value at a specified x and y coordinate in the Map
    double valAt(int x, int y);


    // Method to display the Map, optionally with a heat map
    cv::Mat display(bool show_heat_map);


    // Method to save the Map to a file
    bool save(const std::string &filename);


    // Static method to load a Map from a file and return a shared pointer to it
    static Map::Ptr load(const std::string &filename);
};

#endif //ROBOTNAVIGATION_MAP_H