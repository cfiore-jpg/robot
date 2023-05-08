//
// Created by Cameron Fiore on 5/2/23.
//
#include <vector>
#include <iostream>
#include <cmath>

#ifndef ROBOTNAVIGATION_OBJECT_H
#define ROBOTNAVIGATION_OBJECT_H

struct Coord {
public:
    int x;
    int y;

public:
    Coord(int i, int j) : x(i), y(j) {}

    [[nodiscard]] double dist(const Coord& c) const;

    [[nodiscard]] std::vector<Coord> surrounding(int max_x, int max_y) const;
};




class Object : public std::enable_shared_from_this<Object> {

protected:
    Coord coord;

public:
    const double radius;
    using Ptr = std::shared_ptr<Object>;

public:
    Object(int i, int j, double r) : coord(Coord(i, j)), radius(r) {}

    static Object::Ptr createObject(int x, int y, double radius);

    double dist(const Coord& c) const;

    [[nodiscard]] int x() const;

    [[nodiscard]] int y() const;

};

#endif //ROBOTNAVIGATION_OBJECT_H