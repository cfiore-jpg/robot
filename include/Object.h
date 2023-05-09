//
// Created by Cameron Fiore on 5/2/23.
//
#include <vector>
#include <iostream>
#include <cmath>


#ifndef ROBOTNAVIGATION_OBJECT_H
#define ROBOTNAVIGATION_OBJECT_H


struct Coord {
    int x;
    int y;


    Coord(int i, int j) : x(i), y(j) {}


    // Calculate the distance between two coordinates
    [[nodiscard]] double dist(const Coord& c) const;


    // Return a vector of all surrounding coordinates within a given max_x and max_y range
    [[nodiscard]] std::vector<Coord> surrounding(int max_x, int max_y) const;
};


class Object : public std::enable_shared_from_this<Object> {
protected:
    Coord coord;


public:
    const double radius;
    using Ptr = std::shared_ptr<Object>;


    Object(int i, int j, double r) : coord(Coord(i, j)), radius(r) {}


    // Create a shared pointer to an Object instance
    static Object::Ptr createObject(int x, int y, double radius);


    // Calculate the distance between an Object's coordinate and a given coordinate
    double dist(const Coord& c) const;


    // Return the x-coordinate of the Object
    [[nodiscard]] int x() const;


    // Return the y-coordinate of the Object
    [[nodiscard]] int y() const;
};


#endif //ROBOTNAVIGATION_OBJECT_H