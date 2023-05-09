//
// Created by Cameron Fiore on 4/29/23.
//

#ifndef ROBOTNAVIGATION_ROBOT_H
#define ROBOTNAVIGATION_ROBOT_H

#include "Map.h"
#include <utility>
#include <memory>

struct Qobject {

public:
    double space;
    double dist;
    Coord coord;

public:
    Qobject(double s, double d, Coord c): space(s), dist(d), coord(c) {}
};


class Robot : public Object {

private:
    Map::Ptr map_;
    Coord target_;

public:
    using Ptr = std::shared_ptr<Robot>;


public:

    explicit Robot(double r);

    static Robot::Ptr create(double r);

    bool setStart(int x, int y);

    bool setTarget(int x, int y);

    bool giveMap(Map::Ptr m);

    cv::Mat showOnMap(bool show_heat_map,
                      const std::vector<std::vector<Coord>>& paths = {},
                      const std::vector<cv::Vec3b>& colors = {});

    std::vector<Coord> pathFind(double lambda, bool save, const std::string& fn = "output");

    void printParameters() const;

















public:
    cv::Mat showOnMapV1(bool show_heat_map,
                             const std::vector<std::vector<Coord>>& paths = {},
                             const std::vector<cv::Vec3b>& colors = {});
    std::vector<Coord> pathFindV1(bool save, const std::string& fn = "output");

};


#endif //ROBOTNAVIGATION_ROBOT_H
