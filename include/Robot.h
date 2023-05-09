

#ifndef ROBOTNAVIGATION_ROBOT_H
#define ROBOTNAVIGATION_ROBOT_H


#include "Map.h"
#include <utility>
#include <memory>




class Robot : public Object {
private:
    Map::Ptr map_;   // A shared pointer to the map the robot operates on
    Coord target_;   // The target location on the map for the robot


public:
    using Ptr = std::shared_ptr<Robot>;   // A shared pointer to a robot object


    explicit Robot(double r);   // Constructor to create a robot object with a given radius


    static Robot::Ptr create(double r);   // A static factory method to create a robot object with a given radius


    bool setStart(int x, int y);   // A method to set the starting position of the robot on the map


    bool setTarget(int x, int y);   // A method to set the target position of the robot on the map


    bool giveMap(Map::Ptr m);   // A method to provide the robot with a map to operate on


    cv::Mat showOnMap(bool show_heat_map,
                      const std::vector<std::vector<Coord>> &paths = {},
                      const std::vector<cv::Vec3b> &colors = {});   // A method to display the robot on the map


    std::vector<Coord> pathFind(double lambda, bool save, const std::string &fn = "output");   // A method to find a path for the robot on the map


    void printParameters() const;   // A method to print the parameters of the robot object
};


#endif //ROBOTNAVIGATION_ROBOT_H