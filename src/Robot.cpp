//
// Created by Cameron Fiore on 4/29/23.
//

#include <utility>

#include "../include/Robot.h"

int INFi = std::numeric_limits<int>::max();
double INFd = std::numeric_limits<double>::max();

struct Qcomp {
public:
    double l;
    explicit Qcomp(double l_value) : l(l_value) {}
    bool operator()(const Qobject &q1, const Qobject &q2) const {
        return (l * q1.space - (1. - l) * std::log(q1.dist + 0.01)) <
               (l * q2.space - (1. - l) * std::log(q2.dist + 0.01));
    }
};

Robot::Robot(double r) : Object(0, 0, r), target_(Coord(0, 0)), map_(nullptr) {
    if (radius <= 0) {
        throw std::invalid_argument("Radius must be greater than 0...");
    }
}

Robot::Ptr Robot::create(double r) {
    return std::make_shared<Robot>(r);
}

bool Robot::setStart(int x, int y) {
    coord = {x, y};
    return true;
}

bool Robot::setTarget(int x, int y) {
    target_ = {x, y};
    return true;
}

bool Robot::giveMap(Map::Ptr m) {
    map_ = std::move(m);
    std::cerr << "Warning: Map has changed. Make sure to set robot start and target appropriately.\n";
    return true;
}

cv::Mat Robot::showOnMap(bool show_heat_map,
                         const std::vector<std::vector<Coord>>& paths,
                         const std::vector<cv::Vec3b>& colors) {

    if (map_ == nullptr) {
        std::cerr << "No map on which to show robot.\n";
        return {};
    }
    if (coord.x < 0 || coord.x >= map_->rows || coord.y < 0 || coord.y >= map_->cols) {
        std::cerr << "Start location is out of bounds for given map. Set start in bounds before displaying.\n";
        return {};
    }
    if (target_.x < 0 || target_.x >= map_->rows || target_.y < 0 || target_.y >= map_->cols) {
        std::cerr << "Target is out of bounds for given map. Set target in bounds before displaying.\n";
        return {};
    }

    const int rows = map_->rows;
    const int cols = map_->cols;

    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Vec3b(0, 0, 0));

    if (show_heat_map) {
        const double max_dist = std::max(rows / 2, cols / 2);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, uchar(std::min(255., 255. * (map_->valAt(i, j) / max_dist))),
                                                      0);
            }
        }
    }

    for (const auto &obj: map_->getObstacles()) {
        const int lx = int(ceil(obj->x() - obj->radius));
        const int ux = int(floor(obj->x() + obj->radius));
        for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
            const int ly = int(ceil(obj->y() - sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            const int uy = int(floor(obj->y() + sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
            }
        }
    }

    for (int p = 0; p < paths.size(); p++) {
        const cv::Vec3b &color = colors[p];
        for (const auto &coord: paths[p]) {
            image.at<cv::Vec3b>(coord.x, coord.y) = color;
        }
    }

    int lx = int(ceil(coord.x - radius));
    int ux = int(floor(coord.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        const int ly = int(ceil(coord.y - sqrt(radius * radius - (i - coord.x) * (i - coord.x))));
        const int uy = int(floor(coord.y + sqrt(radius * radius - (i - coord.x) * (i - coord.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 165, 255);
        }
    }

    lx = int(ceil(target_.x - radius));
    ux = int(floor(target_.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        const int ly = int(ceil(target_.y - sqrt(radius * radius - (i - target_.x) * (i - target_.x))));
        const int uy = int(floor(target_.y + sqrt(radius * radius - (i - target_.x) * (i - target_.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 0, 128);
        }
    }

    cv::Mat resized_image;
    int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    return resized_image;
}



[[nodiscard]] std::vector<Coord> Robot::pathFind(double lambda, bool save, const std::string& fn) {
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

    std::vector<Coord> links(rows * cols, {-1, -1});

    std::vector<bool> visited(rows * cols, false);
    visited[coord.x * cols + coord.y] = true;

    std::priority_queue<Qobject, std::vector<Qobject>, Qcomp> pq((Qcomp(lambda)));
    pq.emplace(0, 0, coord);

    std::vector<Coord> search;
    search.reserve(rows * cols);

    auto start = std::chrono::high_resolution_clock::now();
    while (!pq.empty()) {
        const auto cur_c = pq.top().coord;
        pq.pop();

        if (save) {
            search.push_back(cur_c);
        }

        if (cur_c.x == target_.x && cur_c.y == target_.y) {
            break;
        }

        for (const auto &next_c: cur_c.surrounding(rows, cols)) {
            const auto next_key = next_c.x * cols + next_c.y;
            const auto next_s = map_->valAt(next_c);
            if (!visited[next_key] && next_s >= radius) {
                pq.emplace(next_s / max_s, next_c.dist(target_) / max_d, next_c);
                visited[next_key] = true;
                links[next_key] = cur_c;
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Path time: " << duration.count() << " milliseconds\n";

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






















cv::Mat Robot::showOnMapV1(bool show_heat_map,
                         const std::vector<std::vector<Coord>>& paths,
                         const std::vector<cv::Vec3b>& colors) {

    const int rows = map_->rows_d;
    const int cols = map_->cols_d;

    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Vec3b(0, 0, 0));

    if (show_heat_map) {
        const double max_dist = std::max(rows / 2, cols / 2);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, uchar(std::min(255., 255. * (map_->valAtV1(i, j) / max_dist))),
                                                      0);
            }
        }
    }

    for (const auto &obj: map_->getObstacles()) {
        const int lx = int(ceil(obj->x() - obj->radius));
        const int ux = int(floor(obj->x() + obj->radius));
        for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
            const int ly = int(ceil(obj->y() - sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            const int uy = int(floor(obj->y() + sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
            }
        }
    }

    for (int p = 0; p < paths.size(); p++) {
        const cv::Vec3b &color = colors[p];
        for (const auto &coord: paths[p]) {
            image.at<cv::Vec3b>(coord.x, coord.y) = color;
        }
    }

    int lx = int(ceil(coord.x - radius));
    int ux = int(floor(coord.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        const int ly = int(ceil(coord.y - sqrt(radius * radius - (i - coord.x) * (i - coord.x))));
        const int uy = int(floor(coord.y + sqrt(radius * radius - (i - coord.x) * (i - coord.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 165, 255);
        }
    }

    lx = int(ceil(target_.x - radius));
    ux = int(floor(target_.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        const int ly = int(ceil(target_.y - sqrt(radius * radius - (i - target_.x) * (i - target_.x))));
        const int uy = int(floor(target_.y + sqrt(radius * radius - (i - target_.x) * (i - target_.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 0, 128);
        }
    }

    cv::Mat resized_image;
    int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    return resized_image;
}




struct Qold {

public:
    double space;
    Coord coord;

public:
    Qold(double s, Coord c): space(s), coord(c) {}
};


struct Oldcomp {
public:
    bool operator()(const Qold &q1, const Qold &q2) const {
        return q1.space < q2.space;
    }
};



std::vector<Coord> Robot::pathFindV1(bool save, const std::string& fn) {

    const int rows = map_->rows_d;
    const int cols = map_->cols_d;

    std::vector<Coord> links(rows * cols, {-1, -1});

    std::vector<bool> visited(rows * cols, false);
    visited[coord.x * cols + coord.y] = true;

    std::priority_queue<Qold, std::vector<Qold>, Oldcomp> pq;
    pq.emplace(0, coord);

    std::vector<Coord> search;
    search.reserve(rows * cols);


    auto start = std::chrono::high_resolution_clock::now();
    while (!pq.empty()) {
        const auto cur_c = pq.top().coord;
        pq.pop();

        if (save) {
            search.push_back(cur_c);
        }

        if (cur_c.x == target_.x && cur_c.y == target_.y) {
            break;
        }

        for (const auto &next_c: cur_c.surrounding(rows, cols)) {
            const auto next_key = next_c.x * cols + next_c.y;
            const auto next_s = map_->valAtV1(next_c.x, next_c.y);
            if (!visited[next_key] && next_s >= radius) {
                pq.emplace(next_s, next_c);
                visited[next_key] = true;
                links[next_key] = cur_c;
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Path time: " << duration.count() << " milliseconds\n";

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


    if (save) {

        const int div = int(std::max(double(search.size()) / double(100), 1.));
        cv::VideoWriter video(fn,
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, showOnMapV1(false).size());
        std::vector<Coord> where_ive_been;
        where_ive_been.reserve(search.size());
        int count = 0;
        for (int i = 0; i < search.size(); i += 1) {
            where_ive_been.push_back(search[i]);
            if (count == 0) {
                cv::Mat frame = showOnMapV1(true, {where_ive_been}, {cv::Vec3b(0, 0, 255.)});
                video.write(frame);
                count = div;
            }
            count--;
        }
        cv::Mat frame = showOnMapV1(true, {path}, {cv::Vec3b(0, 0, 255.)});
        video.write(frame);
    }

    return path;
}