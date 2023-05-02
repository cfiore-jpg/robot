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
    std::vector<Coord> sur;
    sur.reserve(8);
    for (int x_next = std::max(0, x - 1); x_next < std::min(max_x, x + 2); ++x_next) {
        for (int y_next = std::max(0, y - 1); y_next < std::min(max_y, y + 2); ++y_next) {
            if (!(x_next == x && y_next == y)) {
                sur.emplace_back(x_next, y_next);
            }
        }
    }
    return sur;
}




//// Object
Object::Object(int i, int j, double r) : coord(Coord(i, j)), radius(r) {}

Object::Ptr Object::createObject(int x, int y, double radius) {
    return std::make_shared<Object>(x, y, radius);
}

int Object::x() const {
    return coord.x;
}

int Object::y() const {
    return coord.y;
}
