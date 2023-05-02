#include "../include/Map.h"


//// Map

Map::Map(int r, int c) : rows(r), cols(c) {
    if (r < 1) {
        throw std::invalid_argument("rows must be greater than or equal to 1");
    }
    if (c < 1) {
        throw std::invalid_argument("cols must be greater than or equal to 1");
    }
    map_.resize(rows, std::vector<int>(cols, 0));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (i == 0 || j == 0 || i == rows - 1 || j == cols - 1) map_[i][j] = 1;
        }
    }
}

Map::Ptr Map::createMap(int r, int c) {
    return std::make_shared<Map>(r, c);
}

int Map::numObjects() {
    return int(objects.size());
}

cv::Mat Map::display() {

    int mult = std::max(1000 / std::max(rows, cols), 1);

    const int _rows = rows * mult;
    const int _cols = cols * mult;

    cv::Mat image(rows * mult, cols * mult, CV_8UC3);

    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
            if (map_[i / mult][j / mult]) {
                image.at<cv::Vec3b>(i, j)[0] = 255;
            }
        }
    }
    return image;
}

cv::Mat Map::display(const std::vector<std::vector<Coord>>& paths,
                  const std::vector<cv::Vec3b>& colors) {

    int mult = std::max(1000 / std::max(rows, cols), 1);

    const int _rows = rows * mult;
    const int _cols = cols * mult;

    cv::Mat image(_rows, _cols, CV_8UC3);

    for (int i = 0; i < _rows; i++) {
        for (int j = 0; j < _cols; j++) {
            if (map_[i / mult][j / mult]) {
                image.at<cv::Vec3b>(i, j)[0] = 255;
            }
        }
    }

    for (int p = 0; p < paths.size(); p++) {
        const cv::Vec3b& color = colors[p];
        for (const auto &coord: paths[p]) {
            assert(coord.x >= 0 && coord.y < rows && coord.y >= 0 && coord.y < cols);
            for (int i = coord.x * mult; i < (coord.x + 1) * mult; i++) {
                for (int j = coord.y * mult; j < (coord.y + 1) * mult; j++) {
                    image.at<cv::Vec3b>(i, j) = color;
                }
            }
        }
    }

    return image;
}

bool Map::addObject(const Object::Ptr &object) {

    const int c_x = object->x();
    const int c_y = object->y();
    const double c_r = object->radius;

    if (c_x < 0 || c_x > rows - 1 || c_y < 0 || c_y > cols - 1) {
        std::cerr << "Object location is out of bounds..." << std::endl;
        return false;
    }

    int lx = int(ceil(c_x - c_r));
    int ux = int(floor(c_x + c_r));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        int ly = int(ceil(c_y - sqrt(c_r * c_r - (i - c_x) * (i - c_x))));
        int uy = int(floor(c_y + sqrt(c_r * c_r - (i - c_x) * (i - c_x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            map_[i][j] += 1;
        }
    }

    objects.insert(object);
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

    auto iter = objects.find(object);
    if (iter == objects.end()) {
        std::cerr << "This map does not contain that object..." << std::endl;
        return false;
    }

    const Object::Ptr& to_delete = *iter;
    const int c_x = to_delete->x();
    const int c_y = to_delete->y();
    const double c_r = to_delete->radius;

    int lx = int(ceil(c_x - c_r));
    int ux = int(floor(c_x + c_r));
    for (int i = std::max(0, lx); i <= std::min(ux, rows - 1); i++) {
        int ly = int(ceil(c_y - sqrt(c_r * c_r - (i - c_x) * (i - c_x))));
        int uy = int(floor(c_y + sqrt(c_r * c_r - (i - c_x) * (i - c_x))));
        for (int j = std::max(0, ly); j <= std::min(uy, cols - 1); j++) {
            map_[i][j] -= 1;
        }
    }

    objects.erase(iter);
    return true;
}

bool Map::removeObject(int x, int y, double r) {

    Object::Ptr to_delete = nullptr;
    for (const auto & obj : objects) {
        if (obj->x() == x && obj->y() == y && std::fabs(obj->radius - r) <= 0.000001) {
            to_delete = obj;
            break;
        }
    }
    if (to_delete == nullptr) {
        std::cerr << "This map does not contain an object with those parameters..." << std::endl;
        return false;
    }

    return removeObject(to_delete);
}

void Map::clearMap() {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (i == 0 || j == 0 || i == rows - 1 || j == cols - 1) {
                map_[i][j] = 1;  // Edge element, set value to 1
            } else {
                map_[i][j] = 0;  // Non-edge element, set value to 0
            }
        }
    }
    objects.clear();
}

[[nodiscard]] double Map::valAt(const Coord& c) const {

    if (c.x < 0 || c.x > rows - 1 || c.y < 0 || c.y > cols - 1) {
        std::cerr << "Coordinate out of bounds..." << std::endl;
        return -1;
    }

    return map_[c.x][c.y];
}

bool Map::save(const std::string& filename) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return false;
    }
    outfile << rows << " " << cols << std::endl;
    for (const auto& obj : objects) {
        outfile << obj->x() << " " << obj->y() << " " << std::setprecision(10) << obj->radius << std::endl;
    }
    return true;
}

Map::Ptr Map::load(const std::string& filename) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for reading." << std::endl;
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