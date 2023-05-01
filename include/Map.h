#include <vector>
#include <iostream>
#include <queue>
#include <iomanip>
#include <cmath>
#include <unordered_set>
#include <fstream>
#include <sstream>

#ifndef ROBOTNAVIGATION_MAP_H
#define ROBOTNAVIGATION_MAP_H

struct Coord {
public:
    int x;
    int y;

public:
    Coord(int i, int j);

};





struct Object {
public:
    const Coord coord;
    const double radius;
    using Ptr = std::shared_ptr<Object>;

public:
    Object(int x, int y, double radius);

    static Object::Ptr createObject(int x, int y, double radius);

    [[nodiscard]] double distanceFrom(double x, double y) const;

};





class Map {

private:
    std::vector<std::vector<double>> map_;
    std::unordered_set<Object::Ptr> objects;

public:
    const int rows;
    const int cols;
    using Ptr = std::shared_ptr<Map>;


private:
    void reset(bool clear_objects);

public:
    Map(int r, int c);

    static Map::Ptr createMap(int r, int c);

    void print(const std::vector<Coord> &path) const;

    bool addObject(const Object::Ptr& object);

    bool removeObject(const Object::Ptr& object);

    void clearMap();

    [[nodiscard]] double spaceAt(int i, int j) const;

    bool save(const std::string& filename);

    static Map::Ptr load(const std::string& filename);
};



#endif //ROBOTNAVIGATION_MAP_H
