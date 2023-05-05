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


 class Map {

public:
    const int rows;
    const int cols;
    using Ptr = std::shared_ptr<Map>;


private:
    std::unordered_set<Object::Ptr> obstacles;
    std::unordered_set<Object::Ptr> robots;


public:
    Map(int r, int c);

    static Map::Ptr createMap(int r, int c);

    int numObjects();

    cv::Mat display();

    cv::Mat display(const std::vector<std::vector<Coord>>& paths,
                 const std::vector<cv::Vec3b>& colors);

    bool addObject(const Object::Ptr& object);

    Object::Ptr addObject(int x, int y, double r);

    bool removeObject(const Object::Ptr& object);

    bool removeObject(int x, int y, double r);

    void clearMap();

    [[nodiscard]] std::vector<Object::Ptr> getObstacles() const;

    bool save(const std::string& filename);

    static Map::Ptr load(const std::string& filename);

    bool addRobot(const Object::Ptr& bot, bool force);

    bool removeRobot(const Object::Ptr& bot);
};