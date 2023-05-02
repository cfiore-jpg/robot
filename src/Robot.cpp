//
// Created by Cameron Fiore on 4/29/23.
//

#include "../include/Robot.h"

int INFi = std::numeric_limits<int>::max();
double INFd = std::numeric_limits<double>::max();

Qobject::Qobject(double s, double d, Coord c): space(s), dist(d), coord(c) {}



Robot::Robot(int x, int y, double r, Map::Ptr m) : Object(x, y, r), map_(std::move(m)), target_(Coord(x, y)) {
    if (x <= 0 || x >= map_->rows - 1 || y <= 0 || y >= map_->cols - 1) {
        throw std::invalid_argument("Invalid coordinates for Robot: (" + std::to_string(x) + ", " + std::to_string(y) + ")");
    }
    if (nearest(coord, radius) < radius) {
        throw std::invalid_argument("Robot radius " + std::to_string(r) + " too large for given coordinates.");
    }
}


bool Robot::setStart(int x, int y) {
    if (x <= 0 || x >= map_->rows - 1 || y <= 0 || y >= map_->cols - 1) {
        throw std::invalid_argument("Invalid coordinates for Robot: (" + std::to_string(x) + ", " + std::to_string(y) + ")");
    }
    if (nearest(coord, radius) < radius) {
        throw std::invalid_argument("Robot radius " + std::to_string(radius) + " too large for given coordinates.");
    }
    coord = Coord(x, y);
    return true;
}

bool Robot::setTarget(int x, int y) {
    if (x <= 0 || x >= map_->rows - 1 || y <= 0 || y >= map_->cols - 1) {
        std::cerr << "Can't set the target out of bounds..." << std::endl;
        return false;
    }
    target_ = Coord(x, y);
    return true;
}


void Robot::changeMap(Map::Ptr m) {
    map_ = std::move(m);
}



double Robot::nearest(const Coord& c, double r) const {

    std::unordered_set<int> visited;
    visited.insert(c.x * map_->cols + c.y);

    std::queue<Coord> q;
    q.emplace(c);

    while (!q.empty()) {
        auto cur_c = q.front();
        q.pop();

        if (map_->valAt(cur_c) > 0) {
            return cur_c.dist(c);
        } else {
            std::vector<Coord> next = {{std::max(0, cur_c.x - 1),              cur_c.y},
                                       {std::min(cur_c.x + 1, map_->rows - 1), cur_c.y},
                                       {cur_c.x,                               std::min(cur_c.y + 1, map_->cols - 1)},
                                       {cur_c.x,                               std::max(0, cur_c.y - 1)}};
            for (const auto &next_c: next) {
                int next_key = next_c.x * map_->cols + next_c.y;
                double next_d = next_c.dist(c);
                if (visited.find(next_key) == visited.end() && next_d <= r) {
                    visited.insert(next_key);
                    q.emplace(next_c);
                }
            }
        }
    }

    return r;
}



[[nodiscard]] std::vector<Coord> Robot::findSafestPath(double search_radius, bool display) const {

    if (search_radius < radius) {
        std::cerr << "Search radius must be greater than or equal to robot radius... " << std::endl;
        return {};
    }

    double start_space = nearest(coord, search_radius);

    double diag = std::hypot(map_->rows, map_->cols);

    if (start_space < radius) {
        std::cerr << "Robot radius too large start here..." << std::endl;
        return {};
    }

    std::unordered_map<int, Coord> links;
    std::unordered_set<int> visited;

    std::priority_queue<Qobject, std::vector<Qobject>, q_comp> pq;
    pq.emplace(start_space, coord.dist(target_), coord);

    visited.emplace(coord.x * map_->cols + coord.y);

    std::vector<Coord> search;
    std::vector<cv::Mat> vid;

    while (!pq.empty()) {
        auto qobj= pq.top();
        pq.pop();

        const Coord cur_c = qobj.coord;
        const double cur_s = qobj.space;

        if (cur_c.x == target_.x && cur_c.y == target_.y) {
            break;
        }

        auto edges = send(cur_c, std::min(search_radius, cur_s + sqrt(2)));

        search.push_back(qobj.coord);
        vid.push_back(map_->display({search, edges}, {cv::Vec3b(0, 0, 255), cv::Vec3b(0, 255, 255)}));

        for (const auto & next_c : cur_c.surrounding(map_->rows, map_->cols)) {
            int next_key = next_c.x * map_->cols + next_c.y;
            double next_s = recieve(next_c, edges);
            if (visited.find(next_key) == visited.end() && next_s >= radius) {
                pq.emplace(next_s,
                           next_c.dist(target_),
                           next_c);
                visited.emplace(next_key);
                links.insert({next_key, cur_c});
            }
        }
    }


    std::deque<Coord> dq;
    int target_key = target_.x * map_->cols + target_.y;
    if (links.find(target_key) == links.end()) {
        std::cerr << "Impossible to reach target..." << std::endl;
        return {};
    } else {
        auto prev = target_;
        while (!(prev.x == coord.x && prev.y == coord.y)) {
            dq.push_front(prev);
            prev = links.find(prev.x * map_->cols + prev.y)->second;
        }
        dq.push_front(prev);
    }

    auto path = std::vector<Coord>(dq.begin(), dq.end());

    for(int i = 0; i < 40; i++) {
        vid.push_back(map_->display({path}, {cv::Vec3b(0, 0, 255)}));
    }

    cv::VideoWriter video("/Users/cameronfiore/C++/ShieldAI/RobotNavigationDemo/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 20, vid[0].size());

    for (const auto& frame : vid) {
        video.write(frame);
    }

    return path;
}







std::vector<Coord> Robot::send(const Coord& c, double r) const {

    if (map_->valAt(c) > 0) {
        std::cerr << "Can't send radar from inside an object..." << std::endl;
        return {};
    }

    std::unordered_set<int> visited;
    visited.insert(c.x * map_->cols + c.y);

    std::queue<Coord> q;
    q.emplace(c);

    std::vector<Coord> edges;
    edges.reserve(int(floor(3.1415 * r * r)));
    while (!q.empty()) {
        auto cur_c = q.front();
        q.pop();
        if (map_->valAt(cur_c) > 0) {
            edges.push_back(cur_c);
        } else {
            for (const auto &next_c: cur_c.surrounding(map_->rows, map_->cols)) {
                int next_key = next_c.x * map_->cols + next_c.y;
                double next_d = c.dist(next_c);
                if (visited.find(next_key) == visited.end() && next_d <= r) {
                    visited.insert(next_key);
                    q.push(next_c);
                }
            }
        }
    }
    return edges;
}



double Robot::recieve(const Coord& c, const std::vector<Coord>& edges) {
    double nearest = std::numeric_limits<double>::max();
    for(const auto & coord : edges) {
        double dist = c.dist(coord);
        if (dist < nearest) {
            nearest = dist;
        }
    }
    return nearest;
}
