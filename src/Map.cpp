#include "../include/Map.h"


//// Map


Map::Map(int r, int c) : rows(r), cols(c) {
    if (r < 1) {
        throw std::invalid_argument("rows must be greater than or equal to 1");
    }
    if (c < 1) {
        throw std::invalid_argument("cols must be greater than or equal to 1");
    }
    obstacles.reserve(rows * cols);
    heat_map_.resize(rows * cols);
    vert_dist_.resize(rows);
    hor_dist_.resize(cols);
    for (int i = 0; i < rows; i++) {
        vert_dist_[i] = i < rows / 2 ? i : rows - (i + 1);
    }
    for (int j = 0; j < cols; j++) {
        hor_dist_[j] = j < cols / 2 ? j : cols - (j + 1);
    }
}

Map::Ptr Map::createMap(int r, int c) {
    return std::make_shared<Map>(r, c);
}

int Map::numObjects() {
    return int(obstacles.size());
}

bool Map::addObject(const Object::Ptr &object) {

    const int c_x = object->x();
    const int c_y = object->y();
    const double c_r = object->radius;

    if (c_x < 0 || c_x > rows - 1 || c_y < 0 || c_y > cols - 1) {
        std::cerr << "Object location is out of bounds...\n";
        return false;
    } else if (c_r <= 0) {
        std::cerr << "Object must have positive radius...\n";
        return false;
    }

    obstacles.insert(object);

    std::queue<Coord> q;
    q.emplace(c_x, c_y);

    std::vector<bool> visited(rows * cols, false);
    visited[c_x * cols + c_y] = true;

    while (!q.empty()) {
        const Coord cur_c = q.front();
        q.pop();

        const double dist = std::max(0., object->dist(cur_c) - c_r);

        if (dist <= std::min(vert_dist_[cur_c.x], hor_dist_[cur_c.y])) {
            heat_map_[cur_c.x * cols + cur_c.y].emplace(dist, object);
            for (const auto &next_c: cur_c.surrounding(rows, cols)) {
                int next_key = next_c.x * cols + next_c.y;
                if (!visited[next_key]) {
                    visited[next_key] = true;
                    q.push(next_c);
                }
            }
        }
    }

    return true;
}

Object::Ptr Map::addObject(int x, int y, double r) {
    Object::Ptr new_obj = Object::createObject(x, y, r);
    if (addObject(new_obj)) {
        return new_obj;
    } else {
        return nullptr;
    }
}

bool Map::removeObject(const Object::Ptr &object) {
    auto iter = obstacles.find(object);
    if (iter == obstacles.end()) {
        std::cerr << "This map does not contain that object...\n";
        return false;
    }

    obstacles.erase(iter);

    const int c_x = object->x();
    const int c_y = object->y();
    const double c_r = object->radius;

    std::queue<Coord> q;
    q.emplace(c_x, c_y);

    std::vector<bool> visited(rows * cols, false);
    visited[c_x * cols + c_y] = true;

    while (!q.empty()) {
        const Coord cur_c = q.front();
        q.pop();

        const double dist = std::max(0., object->dist(cur_c) - c_r);

        if (dist <= std::min(vert_dist_[cur_c.x], hor_dist_[cur_c.y])) {
            auto &pq = heat_map_[cur_c.x * cols + cur_c.y];
            while (!pq.empty() && obstacles.find(pq.top().obj) == obstacles.end()) {
                pq.pop();
            }
            for (const auto &next_c: cur_c.surrounding(rows, cols)) {
                int next_key = next_c.x * cols + next_c.y;
                if (!visited[next_key]) {
                    visited[next_key] = true;
                    q.push(next_c);
                }
            }
        }
    }
    return true;
}

Object::Ptr Map::removeObject(int x, int y, double r) {
    Object::Ptr to_delete = nullptr;
    for (const auto & obj : obstacles) {
        if (obj->x() == x && obj->y() == y && std::fabs(obj->radius - r) <= 0.000001) {
            to_delete = obj;
            break;
        }
    }
    removeObject(to_delete);
    return to_delete;
}

void Map::clearMap() {
    obstacles.clear();
}

[[nodiscard]] std::vector<Object::Ptr> Map::getObstacles() const {
    return {obstacles.begin(), obstacles.end()};
}

double Map::valAt(const Coord &c) {
    return valAt(c.x, c.y);
}

double Map::valAt(int x, int y) {
    if (x < 0 || x >= rows || y < 0 || y >= cols) {
        return -1;
    }
    auto &pq = heat_map_[x * cols + y];
    return pq.empty() ? std::min(vert_dist_[x], hor_dist_[y]) : pq.top().dist;
}

cv::Mat Map::display(bool show_heat_map) {

    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Vec3b(0, 0, 0));

    if (show_heat_map) {
        const double max_dist = std::max(rows/2, cols/2);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, uchar(std::min(255., 255. * (valAt(i, j) / max_dist))),0);
            }
        }
    }

    for(const auto & obj : obstacles) {
        const int lx = int(ceil(obj->x() - obj->radius));
        const int ux = int(floor(obj->x() + obj->radius));
        for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
            const int ly = int(ceil(obj->y() - sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            const int uy = int(floor(obj->y() + sqrt(obj->radius * obj->radius - (i - obj->x()) * (i - obj->x()))));
            for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(255,0,0);
            }
        }
    }

    cv::Mat resized_image;
    const int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    return resized_image;
}

bool Map::save(const std::string& filename) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return false;
    }
    outfile << rows << " " << cols << std::endl;
    for (const auto& obj : obstacles) {
        outfile << obj->x() << " " << obj->y() << " " << std::setprecision(10) << obj->radius << std::endl;
    }
    return true;
}

Map::Ptr Map::load(const std::string& filename) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for reading.\n";
        return nullptr;
    }

    int r, c;
    infile >> r >> c;
    auto new_map = Map::createMap(r, c);
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int x, y;
        double radius;
        if (iss >> x >> y >> radius) {
            new_map->addObject(x, y, radius);
        }
    }
    return new_map;
}