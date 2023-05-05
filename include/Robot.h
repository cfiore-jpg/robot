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
    Qobject (double s, double d, Coord c);
};


struct Qcomp {
public:
    double l;
    explicit Qcomp(double l_value) : l(l_value) {}
    bool operator()(const Qobject &q1, const Qobject &q2) const {
        double a = l * q1.space - (1. - l) * std::log(q1.dist + 0.0000000001);
        double b = l * q2.space - (1. - l) * std::log(q2.dist + 0.0000000001);
        if (std::fabs(a - b) > 0.0001) {
            return a < b;
        } else {
            return -std::log(q1.dist + 0.0000000001) < -std::log(q2.dist + 0.0000000001);
        }
    }
};


class Robot : public Object {

private:
    Map::Ptr map_;
    std::vector<std::vector<double>> heat_map_;

    Coord target_;
    double search_radius;


public:
    using Ptr = std::shared_ptr<Robot>;


public:

    Robot(int x, int y, double r, double sr);

    static Robot::Ptr create(int x, int y, double r, double sr, const Map::Ptr& m);

    bool setStart(int x, int y);

    bool setTarget(int x, int y);

    bool setSearchRadius(double sr);

    bool changeMap(const Map::Ptr& m);

    cv::Mat showHeatMap();

    cv::Mat showHeatMap(const std::vector<std::vector<Coord>>& paths,
                    const std::vector<cv::Vec3b>& colors);

    std::vector<Coord> findSafestPath(double lambda, bool save, const std::string& fn = "output");

    void printParameters() const;


private:

    bool buildHeatMap();

    void placeObstacle(const Object::Ptr &object);

    double valAt(const Coord& c) const;

};


#endif //ROBOTNAVIGATION_ROBOT_H
