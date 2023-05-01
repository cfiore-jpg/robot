//
// Created by Cameron Fiore on 4/29/23.
//

#ifndef ROBOTNAVIGATION_ROBOT_H
#define ROBOTNAVIGATION_ROBOT_H

#include "Map.h"
#include <utility>


class Robot {
private:
    Map::Ptr map_;
    Coord start_;
    Coord target_;


public:
    const double radius;

public:
    Robot(double r, Map::Ptr  m);

    bool setStart(int x, int y);

    bool setTarget(int x, int y);

    void changeMap(Map::Ptr m);

    [[nodiscard]] std::vector<Coord> findSafestPath(double lambda, bool print) const;
};


struct custom_comp {
    bool operator()(const std::tuple<double, double, int> &t1, const std::tuple<double, double, int> &t2) {
        if (std::get<0>(t1) != std::get<0>(t2)) {
            return std::get<0>(t1) < std::get<0>(t2);
        } else if (std::get<1>(t1) != std::get<1>(t2)) {
            return std::get<1>(t1) > std::get<1>(t2);
        } else {
            return std::get<2>(t1) > std::get<2>(t2);
        }
    }
};


#endif //ROBOTNAVIGATION_ROBOT_H
