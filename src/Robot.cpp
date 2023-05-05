//
// Created by Cameron Fiore on 4/29/23.
//

#include <utility>

#include "../include/Robot.h"

int INFi = std::numeric_limits<int>::max();
double INFd = std::numeric_limits<double>::max();

Qobject::Qobject(double s, double d, Coord c): space(s), dist(d), coord(c) {}

Robot::Robot(int x, int y, double r, double sr) : Object(x, y, r), target_(Coord(x, y)), search_radius(sr) {
    if (search_radius < radius) {
        throw std::invalid_argument("Search radius must be greater than robot radius...");
    }
}

Robot::Ptr Robot::create(int x, int y, double r, double sr, const Map::Ptr& m) {
    Robot::Ptr new_rob = std::make_shared<Robot>(x, y, r, sr);
    m->addRobot(std::dynamic_pointer_cast<Object>(new_rob), false);
    new_rob->map_ = m;
    new_rob->buildHeatMap();
    if (new_rob->valAt(new_rob->coord) < new_rob->radius) {
        m->removeRobot(new_rob);
        throw std::invalid_argument("Robot radius " + std::to_string(r) + " too large for given coordinates.");
    }
    return new_rob;
}

bool Robot::setStart(int x, int y) {
    if (x < 0 || x > map_->rows - 1 || y < 0 || y > map_->cols - 1) {
        throw std::invalid_argument("Invalid coordinates for Robot: (" + std::to_string(x) + ", " + std::to_string(y) + ")");
    }

    if(!flag) {
        if(!buildHeatMap())  {
            return false;
        }
    }

    Coord new_start(x, y);

    if (valAt(new_start) < radius) {
        throw std::invalid_argument("Robot radius " + std::to_string(radius) + " too large for given coordinates.");
    }
    coord = new_start;
    return true;
}

bool Robot::setTarget(int x, int y) {
    if (x < 0 || x > map_->rows - 1 || y < 0 || y > map_->cols - 1) {
        std::cerr << "Can't set the target out of bounds...\n";
        return false;
    }
    target_ = {x, y};
    return true;
}

bool Robot::setSearchRadius(double sr) {
    if (sr < radius) {
        std::cerr << "Search radius must be greater than robot radius...\n";
        return false;
    }
    search_radius = sr;
    return buildHeatMap();
}



bool Robot::changeMap(const Map::Ptr& m) {
    Object::Ptr shared_ptr_this = std::dynamic_pointer_cast<Object>(shared_from_this());
    m->addRobot(shared_ptr_this, true);
    map_->removeRobot(shared_ptr_this);
    map_ = m;
    buildHeatMap();
    std::cerr << "Warning: Map has changed. Make sure to set robot start and target appropriately.\n";
    return true;
}


cv::Mat Robot::showHeatMap() {

    if(!flag) {
        if(!buildHeatMap())  {
            return {};
        }
    }

    int rows = map_->rows;
    int cols = map_->cols;

    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Scalar(0, 0, 0));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(0,uchar(std::min(255., 255. * (heat_map_[i][j] / search_radius))),0);
        }
    }

    for(const auto & obj : map_->getObstacles()) {
        int lx = int(ceil(obj->x() - obj->radius));
        int ux = int(floor(obj->x() + obj->radius));
        for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
            int ly = int(ceil(obj->y() - sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            int uy = int(floor(obj->y() + sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(255,0,0);
            }
        }
    }

    int lx = int(ceil(coord.x - radius));
    int ux = int(floor(coord.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        int ly = int(ceil(coord.y - sqrt(radius * radius - (i - coord.x) * (i -coord.x))));
        int uy = int(floor(coord.y + sqrt(radius * radius - (i - coord.x) * (i - coord.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 165, 255);
        }
    }

    lx = int(ceil(target_.x - radius));
    ux = int(floor(target_.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        int ly = int(ceil(target_.y - sqrt(radius * radius - (i - target_.x) * (i -target_.x))));
        int uy = int(floor(target_.y + sqrt(radius * radius - (i - target_.x) * (i - target_.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 0, 128);
        }
    }

    cv::Mat resized_image;
    int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    return resized_image;
}


cv::Mat Robot::showHeatMap(const std::vector<std::vector<Coord>>& paths,
                    const std::vector<cv::Vec3b>& colors) {

    if(!flag) {
        if(!buildHeatMap())  {
            return {};
        }
    }

    int rows = map_->rows;
    int cols = map_->cols;

    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Scalar(0, 0, 0));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(0,uchar(std::min(255., 255. * (heat_map_[i][j] / search_radius))),0);
        }
    }

    for(const auto & obj : map_->getObstacles()) {
        int lx = int(ceil(obj->x() - obj->radius));
        int ux = int(floor(obj->x() + obj->radius));
        for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
            int ly = int(ceil(obj->y() - sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            int uy = int(floor(obj->y() + sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(255,0,0);
            }
        }
    }

    for (int p = 0; p < paths.size(); p++) {
        const cv::Vec3b &color = colors[p];
        for (const auto &coord: paths[p]) {
            assert(coord.x >= 0 && coord.x < rows && coord.y >= 0 && coord.y < cols);
            image.at<cv::Vec3b>(coord.x, coord.y) = color;
        }
    }

    int lx = int(ceil(coord.x - radius));
    int ux = int(floor(coord.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        int ly = int(ceil(coord.y - sqrt(radius * radius - (i - coord.x) * (i -coord.x))));
        int uy = int(floor(coord.y + sqrt(radius * radius - (i - coord.x) * (i - coord.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 165, 255);
        }
    }

    lx = int(ceil(target_.x - radius));
    ux = int(floor(target_.x + radius));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        int ly = int(ceil(target_.y - sqrt(radius * radius - (i - target_.x) * (i -target_.x))));
        int uy = int(floor(target_.y + sqrt(radius * radius - (i - target_.x) * (i - target_.x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 0, 128);
        }
    }

    cv::Mat resized_image;
    int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    return resized_image;
}



[[nodiscard]] std::vector<Coord> Robot::findSafestPath(double lambda, bool save, const std::string& fn) {

    if (valAt(coord) < radius) {
        std::cerr << "Robot radius "
                     + std::to_string(radius) + " too large for given coordinates. Make sure to set robot start appropriately\n";
        return {};
    }
    if (target_.x < 0 || target_.x > map_->rows - 1 || target_.y < 0 || target_.y > map_->cols - 1) {
        std::cerr << "Can't set the target out of bounds.\n";
        return {};
    }

    if (lambda < 0 || lambda > 1) {
        throw std::invalid_argument("Invalid value for lambda. Valid range is [0, 1]\n");
    }

    if (!flag) {
        if (!buildHeatMap()) {
            return {};
        }
    }

    std::unordered_map<std::size_t, Coord> links;
    std::unordered_set<std::size_t> visited;
    visited.insert(coord.x * map_->cols + coord.y);

    std::priority_queue<Qobject, std::vector<Qobject>, Qcomp> pq((Qcomp(lambda)));
    pq.emplace(0, 0, coord);

    const double diag = std::hypot(map_->rows, map_->cols);

    std::vector<Coord> search;
    search.reserve(map_->rows * map_->cols);
    while (!pq.empty()) {
        const auto qobj = pq.top();
        pq.pop();

        const Coord cur_c = qobj.coord;

        if (save) {
            search.push_back(cur_c);
        }

        if (cur_c.x == target_.x && cur_c.y == target_.y) {
            break;
        }

        for (const auto &next_c: cur_c.surrounding(map_->rows, map_->cols)) {
            const auto next_key = next_c.x * map_->cols + next_c.y;
            const auto next_s = valAt(next_c);
            if (visited.find(next_key) == visited.end() && next_s >= radius) {
                pq.emplace(next_s / search_radius,
                           next_c.dist(target_) / diag,
                           next_c);
                visited.insert(next_key);
                links.emplace(next_key, cur_c);
            }
        }
    }


    std::deque<Coord> dq;
    if (links.find(target_.x * map_->cols + target_.y) == links.end()) {
        std::cerr << "Impossible to reach target...\n";
        return {};
    } else {
        Coord prev = target_;
        while (!(prev.x == coord.x && prev.y == coord.y)) {
            dq.push_front(prev);
            prev = links.find(prev.x * map_->cols + prev.y)->second;
        }
        dq.push_front(prev);
    }
    auto path = std::vector<Coord>(dq.begin(), dq.end());


    if (save > 0) {

        const int div = int(std::max(double(search.size()) / double(100), 1.));
        cv::VideoWriter video(fn,
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, showHeatMap().size());
        std::vector<Coord> where_ive_been;
        where_ive_been.reserve(search.size());
        int count = 0;
        for (int i = 0; i < search.size(); i += 1) {
            where_ive_been.push_back(search[i]);
            if (count == 0) {
                cv::Mat frame = showHeatMap({where_ive_been}, {cv::Vec3b(0, 0, 255.)});
                video.write(frame);
                count = div;
            }
            count--;
        }
        cv::Mat frame = showHeatMap({path}, {cv::Vec3b(0, 0, 255.)});
        video.write(frame);
    }

    return path;
}











double Robot::valAt(const Coord &c) const {
    if (c.x < 0 || c.x > map_->rows - 1 || c.y < 0 && c.y > map_->cols - 1) {
        return -1;
    }
    return heat_map_[c.x][c.y];
}

bool Robot::buildHeatMap() {

    if (map_ == nullptr) {
        std::cerr << "Must assign a map to this robot first...\n";
        return false;
    }

    int rows = map_->rows;
    int cols = map_->cols;

    heat_map_ = std::vector<std::vector<double>> (rows, std::vector<double>(cols, 0));

    std::vector<double> vert_dist(rows);
    std::vector<double> hor_dist(cols);
    for (int i = 0; i < rows; i++) {
        if (i < rows / 2) {
            vert_dist[i] = std::min(double(i), search_radius);
        } else {
            vert_dist[i] = std::min(double(rows - (i + 1)), search_radius);
        }
    }
    for (int j = 0; j < cols; j++) {
        if (j < cols / 2) {
            hor_dist[j] = std::min(double(j), search_radius);
        } else {
            hor_dist[j] = std::min(double(cols - (j + 1)), search_radius);
        }
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            heat_map_[i][j] = std::min({vert_dist[i], hor_dist[j]});
        }
    }

    for (const auto & obj : map_->getObstacles()) {
        placeObstacle(obj);
    }

    flag = true;

    return true;
}

void Robot::placeObstacle(const Object::Ptr &o) {


    std::queue<Coord> q;
    std::unordered_set<int> visited;

    const int c_x = o->x();
    const int c_y = o->y();
    const double c_r = o->radius;

    q.emplace(c_x, c_y);
    visited.insert(c_x * map_->cols + c_y);

    while (!q.empty()) {
        const Coord cur_c = q.front();
        q.pop();

        const double dist = std::max(0., cur_c.dist({c_x, c_y}) - c_r);
        if (dist <= valAt(cur_c) && dist <= search_radius) {
            heat_map_[cur_c.x][cur_c.y] = dist;
            for (const auto &next_c: cur_c.surrounding(map_->rows, map_->cols)) {
                int next_key = next_c.x * map_->cols + next_c.y;
                if (visited.find(next_key) == visited.end()) {
                    visited.insert(next_key);
                    q.push(next_c);
                }
            }
        }
    }
}


void Robot::printParameters() const {
    std::cout << "Start: (" << coord.x << ", " << coord.y << ")\n";
    std::cout << "Target: (" << target_.x << ", " << target_.y << ")\n";
    std::cout << "Radius: " << radius << "\n";
    std::cout << "Search Radius: " << search_radius << "\n";
}
