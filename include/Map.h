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

#endif //ROBOTNAVIGATION_MAP_H

#define MAX_ROWS 100000
#define MAX_COLS 100000

 struct MapItem {
public:
    double dist;
    Object::Ptr obj;

public:
    explicit MapItem(double d, Object::Ptr o): dist(d), obj(std::move(o)) {}
};

struct MapComp {
public:
    bool operator()(const MapItem &m1, const MapItem &m2) const {
        return m1.dist > m2.dist;
    }
};


 class Map {

 public:
     const int rows;
     const int cols;
     using Ptr = std::shared_ptr<Map>;


 private:
     std::vector<double> vert_dist_;
     std::vector<double> hor_dist_;
     std::vector<std::priority_queue<MapItem, std::vector<MapItem>, MapComp>> heat_map_;
     std::unordered_set<Object::Ptr> obstacles;


 public:
     Map(int r, int c);

     static Map::Ptr createMap(int r, int c);

     int numObjects();

     bool addObject(const Object::Ptr &object);

     Object::Ptr addObject(int x, int y, double r);

     bool removeObject(const Object::Ptr &object);

     Object::Ptr removeObject(int x, int y, double r);

     void clearMap();

     [[nodiscard]] std::vector<Object::Ptr> getObstacles() const;

     double valAt(const Coord &c);

     double valAt(int x, int y);

     cv::Mat display(bool show_heat_map);

     bool save(const std::string &filename);

     static Map::Ptr load(const std::string &filename);
 };