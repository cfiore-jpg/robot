//
// Created by Cameron Fiore on 4/29/23.
//

#ifndef ROBOTNAVIGATION_ROBOT_H
#define ROBOTNAVIGATION_ROBOT_H

#include "Map.h"
#include <utility>

struct Qobject {

public:
    double space;
    double dist;
    Coord coord;

public:
    Qobject (double s, double d, Coord c);
};


struct q_comp {

    bool operator()(const Qobject &q1, const Qobject &q2) {
        double a = q1.space / (q1.dist + 0.0000001);
        double b = q2.space / (q2.dist + 0.0000001);
        if (std::fabs(a - b) > 0.000001) {
            return a < b;
        } else {
            return q1.dist > q2.dist;
        }
    }
};



class Robot : public Object {

private:
    Map::Ptr map_;
    Coord target_;


public:
    Robot(int x, int y, double r, Map::Ptr  m);

    bool setStart(int x, int y);

    bool setTarget(int x, int y);

    void changeMap(Map::Ptr m);

    [[nodiscard]] std::vector<Coord> findSafestPath(double search_radius, bool display) const;


private:

    [[nodiscard]] double nearest(const Coord& c, double r) const;

    [[nodiscard]] std::vector<Coord> send(const Coord& c, double r) const;

    [[nodiscard]] static double recieve(const Coord& c, const std::vector<Coord>& edges);

};


#endif //ROBOTNAVIGATION_ROBOT_H
