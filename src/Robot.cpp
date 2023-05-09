//
// Created by Cameron Fiore on 4/29/23.
//


#include <utility>


#include "../include/Robot.h"


// This struct represents an object in a priority queue for path planning
struct Qobject {
    double space; // Available space for movement
    double dist; // Distance from the current node to the target
    Coord coord; // Coordinates of the current node


    // Constructor to initialize the object
    Qobject(double s, double d, Coord c) : space(s), dist(d), coord(c) {}
};


// This struct provides the comparison function for the Qobject priority queue
struct Qcomp {
    double l; // A weight value for the comparison function


    // Constructor to initialize the weight value
    explicit Qcomp(double l_value) : l(l_value) {}


    // The comparison function for the priority queue
    bool operator()(const Qobject &q1, const Qobject &q2) const {
        // Compute the priority value for q1 and q2 and compare them
        return (l * q1.space - (1. - l) * std::log(q1.dist + 0.01)) <
               (l * q2.space - (1. - l) * std::log(q2.dist + 0.01));
    }
};


Robot::Robot(double r) : Object(0, 0, r), target_(Coord(0, 0)), map_(nullptr) {
    // Constructor that sets the initial radius of the robot. If the radius is less than or equal to 0, an exception is thrown.
    if (radius <= 0) {
        throw std::invalid_argument("Radius must be greater than 0...");
    }
}


Robot::Ptr Robot::create(double r) {
    // A static factory method that creates a new Robot object with the specified radius and returns a shared pointer to it.
    return std::make_shared<Robot>(r);
}


bool Robot::setStart(int x, int y) {
    // Sets the starting coordinate of the robot to the specified x and y values and returns true.
    coord = {x, y};
    return true;
}


bool Robot::setTarget(int x, int y) {
    // Sets the target coordinate of the robot to the specified x and y values and returns true.
    target_ = {x, y};
    return true;
}


bool Robot::giveMap(Map::Ptr m) {
    // Sets the map for the robot to the specified map object. A warning message is printed to the console.
    map_ = std::move(m);
    std::cerr << "Warning: Map has changed. Make sure to set robot start and target appropriately.\n";
    return true;
}




// This method shows the robot on a map with the given paths and colors.
// If show_heat_map is true, the map is displayed with a heat map overlay.
// The method returns a resized image of the map with the robot, target and paths displayed.
cv::Mat Robot::showOnMap(bool show_heat_map, const std::vector<std::vector<Coord>> &paths, const std::vector<cv::Vec3b> &colors) {


    // Check if map exists
    if (map_ == nullptr) {
        std::cerr << "No map on which to show robot.\n";
        return {};
    }


    // Check if start location is out of bounds for given map
    if (coord.x < 0 || coord.x >= map_->rows || coord.y < 0 || coord.y >= map_->cols) {
        std::cerr << "Start location is out of bounds for given map. Set start in bounds before displaying.\n";
        return {};
    }


    // Check if target is out of bounds for given map
    if (target_.x < 0 || target_.x >= map_->rows || target_.y < 0 || target_.y >= map_->cols) {
        std::cerr << "Target is out of bounds for given map. Set target in bounds before displaying.\n";
        return {};
    }


    const int rows = map_->rows;
    const int cols = map_->cols;


    // Create an empty image of the map
    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Vec3b(0, 0, 0));


    // If show_heat_map is true, display the map with a heat map overlay
    if (show_heat_map) {
        const double max_dist = std::max(rows / 2, cols / 2);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, uchar(std::min(255., 255. * (map_->valAt(i, j) / max_dist))), 0);
            }
        }
    }


    // Display obstacles on the map
    for (const auto &obj: map_->getObstacles()) {
        cv::circle(image, {obj->y(), obj->x()}, int(obj->radius), cv::Vec3b(255, 0, 0), -1);
    }


    // Display the paths on the map
    for (int p = 0; p < paths.size(); p++) {
        const cv::Vec3b &color = colors[p];
        for (const auto &coord: paths[p]) {
            image.at<cv::Vec3b>(coord.x, coord.y) = color;
        }
    }


    // Display the robot on the map
    cv::circle(image, {coord.y, coord.x}, int(radius), cv::Vec3b(0, 165, 255), -1);


    // Display the target on the map
    cv::circle(image, {target_.y, target_.x}, int(radius), cv::Vec3b(128, 0, 128), -1);


    // Resize the image for display
    cv::Mat resized_image;
    int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);


    // Return the resized image
    return resized_image;
}


// Finds the safest path
[[nodiscard]] std::vector<Coord> Robot::pathFind(double lambda, bool save, const std::string &fn) {
    if (map_ == nullptr) {
        std::cerr << "Give the robot a map before pathFind is called.\n";
        return {};
    }
    if (coord.x < 0 || coord.x >= map_->rows || coord.y < 0 || coord.y >= map_->cols) {
        std::cerr << "Start location is out of bounds for given map. Set start in bounds before pathFind is called.\n";
        return {};
    }
    if (map_->valAt(coord) < radius) {
        std::cerr
                << "Robot can't fit in the start location. Set start somewhere the robot can fit before pathFind is called\n";
        return {};
    }
    if (target_.x < 0 || target_.x >= map_->rows || target_.y < 0 || target_.y >= map_->cols) {
        std::cerr << "Target is out of bounds for given map. Set target in bounds before pathFind is called.\n";
        return {};
    }
    if (lambda < 0 || lambda > 1) {
        throw std::invalid_argument("Invalid value for lambda. Valid range is [0, 1]\n");
    }
    const int rows = map_->rows;
    const int cols = map_->cols;


    const double max_d = std::hypot(rows, cols);
    const double max_s = std::max(rows, cols) / 2.;


    // to store path
    std::vector<Coord> links(rows * cols, {-1, -1});


    // to track visited
    std::vector<bool> visited(rows * cols, false);
    visited[coord.x * cols + coord.y] = true;


    // to decide which coordinate to pop next
    std::priority_queue<Qobject, std::vector<Qobject>, Qcomp> pq((Qcomp(lambda)));
    pq.emplace(0, 0, coord);


    // for display purposes
    std::vector<Coord> search;
    search.reserve(rows * cols);


    while (!pq.empty()) {
        const auto cur_c = pq.top().coord;
        pq.pop();


        if (save) {
            search.push_back(cur_c);
        }


        // break if we've reached target
        if (cur_c.x == target_.x && cur_c.y == target_.y) {
            break;
        }


        // search surrounding nodes
        for (const auto &next_c: cur_c.surrounding(rows, cols)) {
            const auto next_key = next_c.x * cols + next_c.y;
            const auto next_s = map_->valAt(next_c);


            // if we haven't visited this node and the robot can fit
            if (!visited[next_key] && next_s >= radius) {
                pq.emplace(next_s / max_s, next_c.dist(target_) / max_d, next_c);
                visited[next_key] = true;
                links[next_key] = cur_c;
            }
        }
    }


    // backtrace the path
    std::vector<Coord> path;
    path.reserve(rows * cols);
    Coord prev = target_;
    if (links[prev.x * cols + prev.y].x == -1) {
        std::cerr << "Impossible to reach target...\n";
        return {};
    }
    while (prev.x != coord.x || prev.y != coord.y) {
        path.push_back(prev);
        int next_key = prev.x * cols + prev.y;
        prev = links[next_key];
    }
    path.push_back(prev);
    std::reverse(path.begin(), path.end());


    // save journey
    if (save) {


        const int div = int(std::max(double(search.size()) / double(100), 1.));
        cv::VideoWriter video(fn,
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, showOnMap(false).size());
        std::vector<Coord> where_ive_been;
        where_ive_been.reserve(search.size());
        int count = 0;
        for (int i = 0; i < search.size(); i += 1) {
            where_ive_been.push_back(search[i]);
            if (count == 0) {
                cv::Mat frame = showOnMap(true, {where_ive_been}, {cv::Vec3b(0, 0, 255.)});
                video.write(frame);
                count = div;
            }
            count--;
        }
        cv::Mat frame = showOnMap(true, {path}, {cv::Vec3b(0, 0, 255.)});
        video.write(frame);
    }


    return path;
}




void Robot::printParameters() const {
    std::cout << "Start: (" << coord.x << ", " << coord.y << ")\n";
    std::cout << "Target: (" << target_.x << ", " << target_.y << ")\n";
    std::cout << "Radius: " << radius << "\n";
}