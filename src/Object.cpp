//
// Created by Cameron Fiore on 5/2/23.
//

#include "../include/Object.h"


//// Coord
Coord::Coord(int i, int j) : x(i), y(j) {}

double Coord::dist(const Coord& c) const {
    return std::hypot(c.x - x, c.y - y);
}

std::vector<Coord> Coord::surrounding(int max_x, int max_y) const {

    int lbx = std::max(0, x - 1);
    int ubx = std::min(max_x - 1, x + 1);

    int lby = std::max(0, y - 1);
    int uby = std::min(max_y - 1, y + 1);

    std::vector<Coord> sur = {
            {lbx, lby},
            {lbx, y},
            {lbx, uby},
            {x, lby},
            {x, uby},
            {ubx, lby},
            {ubx, y},
            {ubx, uby}};

    return sur;
}




//// Object
Object::Object(int i, int j, double r) : coord(Coord(i, j)), radius(r), flag(true) {}

Object::Ptr Object::createObject(int x, int y, double radius) {
    return std::make_shared<Object>(x, y, radius);
}

int Object::x() const {
    return coord.x;
}

int Object::y() const {
    return coord.y;
}