#include "../include/Map.h"


// Coord
Coord::Coord(int i, int j) : x(i), y(j) {}





// Object
Object::Object(int i, int j, double r) : coord(Coord(i, j)), radius(r) {}

Object::Ptr Object::createObject(int x, int y, double radius) {
    return std::make_shared<Object>(x, y, radius);
}

double Object::distanceFrom(double i, double j) const {
    return std::max(std::hypot(i - coord.x, j - coord.y) - radius, 0.);
}




// Map
void Map::reset(bool clear_objects) {
    std::vector<double> vert_dist(rows);
    std::vector<double> hor_dist(cols);
    for (int i = 0; i < rows; i++) {
        if (i < cols / 2) {
            vert_dist[i] = i;
        } else {
            vert_dist[i] = rows - (i + 1);
        }
    }
    for (int j = 0; j < cols; j++) {
        if (j < cols / 2) {
            hor_dist[j] = j;
        } else {
            hor_dist[j] = cols - (j + 1);
        }
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            map_[i][j] = std::min({vert_dist[i], hor_dist[j]});
        }
    }

    if (clear_objects) objects.clear();
}



Map::Map(int r, int c) : rows(r), cols(c) {
    if (r < 1) {
        throw std::invalid_argument("rows must be greater than or equal to 1");
    }
    if (c < 1) {
        throw std::invalid_argument("cols must be greater than or equal to 1");
    }

    map_.resize(r, std::vector<double>(c, 0));

    std::vector<double> vert_dist(r);
    std::vector<double> hor_dist(c);
    for (int i = 0; i < r; i++) {
        if (i < c / 2) {
            vert_dist[i] = i;
        } else {
            vert_dist[i] = r - (i + 1);
        }
    }
    for (int j = 0; j < c; j++) {
        if (j < c / 2) {
            hor_dist[j] = j;
        } else {
            hor_dist[j] = c - (j + 1);
        }
    }
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            map_[i][j] = std::min({vert_dist[i], hor_dist[j]});
        }
    }

}


Map::Ptr Map::createMap(int r, int c) {
    return std::make_shared<Map>(r, c);
}


void Map::print(const std::vector<Coord> &path) const {

    std::vector<std::vector<std::tuple<int, int, int>>>
            colors(rows, std::vector<std::tuple<int, int, int>>(cols, std::make_tuple(0, 0, 0)));

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int r = std::max(0, std::min(255, static_cast<int>(255 * (7.5 - map_[i][j]) / 7.5)));
            int g = std::max(0, std::min(255, static_cast<int>(255 * map_[i][j] / 7.5)));
            int b = 0;
            if (map_[i][j] == 0) {
                r = 0.;
                g = 0.;
                b = 255.;
            }

            colors[i][j] = std::make_tuple(r, g, b);
        }
    }

    for (const auto &coord: path) {
        colors[coord.x][coord.y] = std::make_tuple(255, 105, 180);
    }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {

            int r = std::get<0>(colors[i][j]);
            int g = std::get<1>(colors[i][j]);
            int b = std::get<2>(colors[i][j]);

            // print colored output
            std::cout << "\033[38;2;" << r << ";" << g << ";" << b << "m";
            std::cout << std::fixed << std::setprecision(1) << std::setw(5) << map_[i][j];
            std::cout << "\033[0m";
        }
        std::cout << std::endl << std::endl;
    }

    std::cout << std::endl << std::endl;
}


bool Map::addObject(const Object::Ptr &object) {
    const int c_x = object->coord.x;
    const int c_y = object->coord.y;

    if (c_x <= 0 || c_x >= rows - 1 || c_y <= 0 || c_y >= cols - 1) {
        std::cerr << "Object location is out of bounds..." << std::endl;
        return false;
    }

    std::queue<Coord> q;
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    q.emplace(c_x, c_y);
    visited[c_x][c_y] = true;

    while (!q.empty()) {
        const auto coord = q.front();
        q.pop();

        const double dist = object->distanceFrom(coord.x, coord.y);
        if (dist <= map_[coord.x][coord.y]) {
            map_[coord.x][coord.y] = dist;
            for (int x_next = std::max(0, coord.x - 1); x_next < std::min(rows, coord.x + 2); ++x_next) {
                for (int y_next = std::max(0, coord.y - 1); y_next < std::min(cols, coord.y + 2); ++y_next) {
                    if (!visited[x_next][y_next]) {
                        q.emplace(x_next, y_next);
                        visited[coord.x][coord.y] = true;
                    }
                }
            }
        }
    }

    objects.insert(object);
    return true;
}


bool Map::removeObject(const Object::Ptr &object) {
    auto iter = objects.find(object);
    if (iter == objects.end()) {
        return false;
    }

    objects.erase(iter);
    reset(false);
    for (const auto &obj: objects) {
        addObject(obj);
    }

    return true;
}

void Map::clearMap() {
    reset(true);
}

double Map::spaceAt(int i, int j) const {
    return map_[i][j];
}

void Map::save(const std::string& filename) {
    std::ofstream outfile(filename);
    outfile << rows << " " << cols << std::endl;
    for (const auto& obj : objects) {
        outfile << obj->coord.x << " " << obj->coord.y << " " << std::setprecision(10) << obj->radius << std::endl;
    }
}

Map::Ptr load(const std::string& filename) {

}