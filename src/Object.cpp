//
// Created by Cameron Fiore on 5/2/23.
//

#include "../include/Object.h"


//// Coord
double Coord::dist(const Coord& c) const {
    return std::hypot(c.x - x, c.y - y);
}

std::vector<Coord> Coord::surrounding(int max_x, int max_y) const {
    int lbx = std::max(0, x - 1);
    int ubx = std::min(max_x, x + 2);
    int lby = std::max(0, y - 1);
    int uby = std::min(max_y, y + 2);
    std::vector<Coord> sur;
    sur.reserve(8);
    for(int i = lbx; i < ubx; i++) {
        for(int j = lby; j < uby; j++) {
            if(i != x || j != y) {
                sur.emplace_back(i, j);
            }
        }
    }
    return sur;
}




//// Object

Object::Ptr Object::createObject(int x, int y, double radius) {
    return std::make_shared<Object>(x, y, radius);
}

double Object::dist(const Coord& c) const {
    return coord.dist(c);
}

int Object::x() const {
    return coord.x;
}

int Object::y() const {
    return coord.y;
}