//
// Created by Cameron Fiore on 4/29/23.
//

#include "../include/Robot.h"


Robot::Robot(double r, Map::Ptr m) : radius(r), map_(std::move(m)), start_(Coord(0, 0)), target_(Coord(0, 0)) {}

bool Robot::setStart(int x, int y) {
    if (x <= 0 || x >= map_->rows - 1 || y <= 0 || y >= map_->cols - 1) {
        std::cout << "Can't start the Robot out of bounds..." << std::endl;
        return false;
    }
    start_ = Coord(x, y);
    return true;
}

bool Robot::setTarget(int x, int y) {
    if (x <= 0 || x >= map_->rows - 1 || y <= 0 || y >= map_->cols - 1) {
        std::cout << "Can't set the target out of bounds..." << std::endl;
        return false;
    }
    target_ = Coord(x, y);
    return true;
}


void Robot::changeMap(Map::Ptr m) {
    map_ = std::move(m);
}

[[nodiscard]] std::vector<Coord> Robot::findSafestPath(double lambda, bool print) const {

    if (lambda < 0 || lambda > 1) {
        std::cout << "Lambda must be between 0 and 1..." << std::endl;
        return {};
    }

    if (map_->spaceAt(start_.x, start_.y) < radius) {
        std::cout << "Robot cannot fit in start location. Change the start location to a safer area..." << std::endl;
        return {};
    }

    if (map_->spaceAt(target_.x, target_.y) < radius) {
        std::cout << "Impossible to reach target..." << std::endl;
        return {};
    }

    std::vector<std::vector<Coord>> links(map_->rows, std::vector<Coord>(map_->cols, Coord(-1, -1)));
    std::vector<std::vector<bool>> visited(map_->rows, std::vector<bool>(map_->cols, false));

    std::priority_queue<std::tuple<double, double, int>, std::vector<std::tuple<double, double, int>>, custom_comp> pq;
    pq.emplace(map_->spaceAt(start_.x, start_.y), 0, start_.x * map_->cols + start_.y);
    visited[start_.x][start_.y] = true;

    while (!pq.empty()) {
        auto point = pq.top();
        pq.pop();
        int x = std::get<2>(point) / map_->cols;
        int y = std::get<2>(point) % map_->cols;

        if (x == target_.x && y == target_.y) {
            break;
        }

        for (int x_next = std::max(0, x - 1); x_next < std::min(map_->rows, x + 2); ++x_next) {
            for (int y_next = std::max(0, y - 1); y_next < std::min(map_->cols, y + 2); ++y_next) {
                if (!visited[x_next][y_next] && map_->spaceAt(x_next, y_next) - radius >= 0) {
                    double safety = map_->spaceAt(x_next, y_next) - radius;
                    double dist_to_target = std::hypot(target_.x - x_next, target_.y - y_next);
                    double score = safety * (1 - lambda) - lambda * dist_to_target ;

                    pq.emplace(score, dist_to_target, x_next * map_->cols + y_next);
                    visited[x_next][y_next] = true;
                    links[x_next][y_next] = Coord(x, y);
                }
            }
        }
    }

    std::deque<Coord> dq;
    if (links[target_.x][target_.y].x == -1 || links[target_.x][target_.y].y == -1) {
        std::cout << "Impossible to reach target..." << std::endl;
        return {};
    } else {
        auto prev = target_;
        while (!(prev.x == start_.x && prev.y == start_.y)) {
            dq.push_front(prev);
            prev = links[prev.x][prev.y];
        }
        dq.push_front(prev);
    }

    auto path = std::vector<Coord>(dq.begin(), dq.end());

    if (print) {
        map_->print(path);
    }

    return path;
}

