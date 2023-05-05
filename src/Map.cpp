#include "../include/Map.h"


//// Map

Map::Map(int r, int c) : rows(r), cols(c) {
    if (r < 1) {
        throw std::invalid_argument("rows must be greater than or equal to 1");
    }
    if (c < 1) {
        throw std::invalid_argument("cols must be greater than or equal to 1");
    }
}

Map::Ptr Map::createMap(int r, int c) {
    return std::make_shared<Map>(r, c);
}

int Map::numObjects() {
    return int(obstacles.size());
}

cv::Mat Map::display() {

    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Scalar(0, 0, 0));
    for(const auto & obj : obstacles) {
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

    cv::Mat resized_image;
    int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    return resized_image;
}

cv::Mat Map::display(const std::vector<std::vector<Coord>>& paths,
                  const std::vector<cv::Vec3b>& colors) {

    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Scalar(0, 0, 0));
    for(const auto & obj : obstacles) {
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
            assert(coord.x >= 0 && coord.y < rows && coord.y >= 0 && coord.y < cols);
            image.at<cv::Vec3b>(coord.x, coord.y) = color;
        }
    }

    cv::Mat resized_image;
    int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    return resized_image;
}

bool Map::addObject(const Object::Ptr &object) {

    const int c_x = object->x();
    const int c_y = object->y();
    const double c_r = object->radius;

    if (c_x < 0 || c_x > rows - 1 || c_y < 0 || c_y > cols - 1) {
        throw std::invalid_argument("Object location is out of bounds...\n");
    }

    obstacles.insert(object);

    for(auto & rob : robots) {
        rob->flag = false;
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
        throw std::invalid_argument("This map does not contain that object...\n");
    }

    obstacles.erase(iter);

    for(auto & rob : robots) {
        rob->flag = false;
    }

    return true;
}

bool Map::removeObject(int x, int y, double r) {

    Object::Ptr to_delete = nullptr;
    for (const auto & obj : obstacles) {
        if (obj->x() == x && obj->y() == y && std::fabs(obj->radius - r) <= 0.000001) {
            to_delete = obj;
            break;
        }
    }
    if (to_delete == nullptr) {
        throw std::invalid_argument("This map does not contain an object with those parameters...\n");
        return false;
    }

    return removeObject(to_delete);
}

void Map::clearMap() {
    obstacles.clear();
    for(auto & rob : robots) {
        rob->flag = false;
    }
}

[[nodiscard]] std::vector<Object::Ptr> Map::getObstacles() const {
    return {obstacles.begin(), obstacles.end()};
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

bool Map::addRobot(const Object::Ptr& bot, bool force) {

    const int c_x = bot->x();
    const int c_y = bot->y();

    if (c_x < 0 || c_x > rows - 1 || c_y < 0 || c_y > cols - 1) {
        if (!force) {
            throw std::invalid_argument("Robot location is out of bounds...\n");
        } else {
            std::cerr << "Robot location is out of bounds...\n";
        }
    }

    robots.insert(bot);

    return true;

}

bool Map::removeRobot(const Object::Ptr& bot) {
    auto iter = robots.find(bot);
    if (iter == robots.end()) {
        std::cerr << "This map does not contain that robot...\n";
        return false;
    }
    robots.erase(iter);
    return true;
}