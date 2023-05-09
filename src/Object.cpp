//
// Created by Cameron Fiore on 5/2/23.
//


#include "../include/Object.h"


// Calculate the Euclidean distance between two coordinates
double Coord::dist(const Coord& c) const {
    return std::hypot(c.x - x, c.y - y);
}


// Return a vector of all surrounding coordinates within a given max_x and max_y range
std::vector<Coord> Coord::surrounding(int max_x, int max_y) const {
    // Define the lower and upper bounds for the x and y coordinates of the surrounding coordinates
    int lbx = std::max(0, x - 1);
    int ubx = std::min(max_x, x + 2);
    int lby = std::max(0, y - 1);
    int uby = std::min(max_y, y + 2);


    // Create an empty vector to hold the surrounding coordinates
    std::vector<Coord> sur;
    sur.reserve(8);


    // Iterate over the surrounding coordinates and add them to the vector
    for (int i = lbx; i < ubx; i++) {
        for (int j = lby; j < uby; j++) {
            if (i != x || j != y) {
                sur.emplace_back(i, j);
            }
        }
    }


    return sur;
}


// Create a shared pointer to an Object instance
Object::Ptr Object::createObject(int x, int y, double radius) {
    return std::make_shared<Object>(x, y, radius);
}


// Calculate the distance between an Object's coordinate and a given coordinate
double Object::dist(const Coord& c) const {
    return coord.dist(c);
}


// Return the x-coordinate of the Object
int Object::x() const {
    return coord.x;
}


// Return the y-coordinate of the Object
int Object::y() const {
    return coord.y;
}
