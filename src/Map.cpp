#include "../include/Map.h"




// This is the constructor of the Map class that initializes the Map object with the given number of rows and columns.
Map::Map(int r, int c) : rows(r), cols(c) {
// Ensure that the number of rows and columns are valid
    if (r < 1) {
        throw std::invalid_argument("rows must be greater than or equal to 1");
    }
    if (c < 1) {
        throw std::invalid_argument("cols must be greater than or equal to 1");
    }


// Reserve memory for the obstacles set
    obstacles.reserve(rows * cols);


// Resize the heat_map_ and the distance vectors to the appropriate size
    heat_map_.resize(rows * cols);
    vert_dist_.resize(rows);
    hor_dist_.resize(cols);


// Calculate the distance from the center for each row and column
    for (int i = 0; i < rows; i++) {
        vert_dist_[i] = i < rows / 2 ? i : rows - (i + 1);
    }
    for (int j = 0; j < cols; j++) {
        hor_dist_[j] = j < cols / 2 ? j : cols - (j + 1);
    }
}


// This static function returns a shared pointer to a newly created Map object with the given number of rows and columns.
Map::Ptr Map::createMap(int r, int c) {
    return std::make_shared<Map>(r, c);
}


// This function returns the number of objects in the obstacles set.
int Map::numObjects() {
    return int(obstacles.size());
}


/**
* Adds an object to the map.
*
* @param object A shared pointer to the object to add.
* @return true if the object was added successfully, false otherwise.
*/
bool Map::addObject(const Object::Ptr &object) {
    const int c_x = object->x();
    const int c_y = object->y();
    const double c_r = object->radius;


    // Check if object location is within the map boundaries.
    if (c_x < 0 || c_x > rows - 1 || c_y < 0 || c_y > cols - 1) {
        std::cerr << "Object location is out of bounds...\n";
        return false;
    }


        // Check if object has positive radius.
    else if (c_r <= 0) {
        std::cerr << "Object must have positive radius...\n";
        return false;
    }


    // Add the object to the map.
    obstacles.insert(object);


    // Create a queue for BFS.
    std::queue<Coord> q;
    q.emplace(c_x, c_y);


    // Create a visited array to keep track of visited cells.
    std::vector<bool> visited(rows * cols, false);
    visited[c_x * cols + c_y] = true;


    // Traverse the map using BFS.
    while (!q.empty()) {
        const Coord cur_c = q.front();
        q.pop();


        // Calculate the distance between the object and the current cell.
        const double dist = std::max(0., object->dist(cur_c) - c_r);


        // Check if the distance is within the range of influence of the object.
        if (dist <= std::min(vert_dist_[cur_c.x], hor_dist_[cur_c.y])) {
            heat_map_[cur_c.x * cols + cur_c.y].emplace(dist, object);


            // Visit all the neighboring cells.
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




// Create an Object with the given coordinates and radius, add it to the Map, and return a shared pointer to it
Object::Ptr Map::addObject(int x, int y, double r) {
    // Create a new Object with the given parameters
    Object::Ptr new_obj = Object::createObject(x, y, r);
    // Attempt to add the new Object to the Map
    if (addObject(new_obj)) {
        // Return a shared pointer to the new Object if it was added successfully
        return new_obj;
    } else {
        // Return a null pointer if the new Object could not be added
        return nullptr;
    }
}




// Remove the given object from the map. Returns true if successful, false otherwise.
bool Map::removeObject(const Object::Ptr &object) {
    auto iter = obstacles.find(object);
    if (iter == obstacles.end()) {  // object not found in the map
        std::cerr << "This map does not contain that object...\n";
        return false;
    }


    obstacles.erase(iter);  // remove the object from the map


    const int c_x = object->x();
    const int c_y = object->y();
    const double c_r = object->radius;


    // update the heat map
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


            // remove all entries in the priority queue that refer to the removed object
            while (!pq.empty() && obstacles.find(pq.top().obj) == obstacles.end()) {
                pq.pop();
            }


            // update the heat map for neighboring cells
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


// Remove the object with the given coordinates and radius from the map. Returns the removed object if successful, nullptr otherwise.
Object::Ptr Map::removeObject(int x, int y, double r) {
    Object::Ptr to_delete = nullptr;
    for (const auto &obj: obstacles) {
        if (obj->x() == x && obj->y() == y && std::fabs(obj->radius - r) <= 0.000001) {
            to_delete = obj;
            break;
        }
    }
    removeObject(to_delete);
    return to_delete;
}


// Remove all objects from the map.
void Map::clearMap() {
    for (const auto & obs : obstacles) {
        removeObject(obs);
    }
}


// Get a vector of shared pointers to all objects in the map.
[[nodiscard]] std::vector<Object::Ptr> Map::getObstacles() const {
    return {obstacles.begin(), obstacles.end()};
}


// Get the heat map value at the given coordinates.
double Map::valAt(const Coord &c) {
    return valAt(c.x, c.y);
}


// Get the heat map value at the given (x, y) coordinates.
double Map::valAt(int x, int y) {
    if (x < 0 || x >= rows || y < 0 || y >= cols) {
        return -1;
    }
    auto &pq = heat_map_[x * cols + y];
    return pq.empty() ? std::min(vert_dist_[x], hor_dist_[y]) : pq.top().dist;
}


// This function generates a CV Mat image representing the current state of the Map
// If show_heat_map is set to true, the heat map will be overlaid on top of the obstacles
cv::Mat Map::display(bool show_heat_map) {


    // Create an empty CV Mat with the dimensions of the Map
    cv::Mat image(rows, cols, CV_8UC3);
    image.setTo(cv::Vec3b(0, 0, 0));


    // If show_heat_map is true, overlay the heat map on top of the black background
    if (show_heat_map) {
        const double max_dist = std::max(rows / 2, cols / 2);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                // The pixel's color is based on the value of the heat map at that location
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, uchar(std::min(255., 255. * (valAt(i, j) / max_dist))), 0);
            }
        }
    }


    // Draw obstacles as blue circles
    for (const auto &obj: obstacles) {
        cv::circle(image, {obj->y(), obj->x()}, int(obj->radius), cv::Vec3b(255, 0, 0), -1);
    }


    // Resize the image for better viewing
    cv::Mat resized_image;
    const int scale = std::max(1000 / std::max(rows, cols), 1);
    cv::resize(image, resized_image, cv::Size(), scale, scale, cv::INTER_NEAREST);


    // Return the resized image
    return resized_image;
}




// Save the current map to a file
bool Map::save(const std::string &filename) {
    // Open output filestream
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return false;
    }
    // Write the dimensions of the map to the file
    outfile << rows << " " << cols << std::endl;
    // Write the obstacle information to the file
    for (const auto &obj: obstacles) {
        outfile << obj->x() << " " << obj->y() << " " << std::setprecision(10) << obj->radius << std::endl;
    }
    return true;
}


// Load a map from a file and return a pointer to it
Map::Ptr Map::load(const std::string &filename) {
    // Open input filestream
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for reading.\n";
        return nullptr;
    }
    // Read in the dimensions of the map
    int r, c;
    infile >> r >> c;
    // Create a new map with the given dimensions
    auto new_map = Map::createMap(r, c);
    // Read in obstacle information and add objects to the map
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
