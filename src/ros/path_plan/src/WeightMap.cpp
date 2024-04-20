// Project Includes
#include "path_plan/WeightMap.hpp"

#include <limits>     //std::numeric_limits
#include <algorithm> //std::min
#include <stdexcept> //std::out_of_range, std::invalid_argument
#include <sstream>     //std::stringstream
#include <iomanip>     //std::setw
#include <algorithm> //std::reverse
#include <cmath>     //std::sqrt
#include <array>
#include <functional>
#include <cassert>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Node Constructor
WeightMap::Node::Node(const mapsize_t x_in,
                      const mapsize_t y_in,
                      const mapsize_t parent_x_in,
                      const mapsize_t parent_y_in,
                      const weight_t weight_in,
                      const fweight_t g_in,
                      const fweight_t h_in) : x(x_in), y(y_in), parent_x(parent_x_in), parent_y(parent_y_in),
                                              weight(weight_in), g(g_in), h(h_in) {};

WeightMap::Node::Node() : x(0), y(0), parent_x(std::numeric_limits<mapsize_t>::max()),
                          parent_y(std::numeric_limits<mapsize_t>::max()), weight(0), g(0), h(0) {};

//---------------	Border Place Methods	----------------------------

bool contains(const BorderPlace &self, const BorderPlace &other) {
    return (static_cast<int>(self) & static_cast<int>(other)) == static_cast<int>(other);
}

bool isValid(const BorderPlace &self) {
    return static_cast<int>(self) < static_cast<int>(BorderPlace::UNKNOWN);
}

const char *to_string(const BorderPlace &self) {
    if (self == BorderPlace::TOP) {
        return "TOP";
    }
    if (self == BorderPlace::BOTTOM) {
        return "BOTTOM";
    }
    if (self == BorderPlace::RIGHT) {
        return "RIGHT";
    }
    if (self == BorderPlace::LEFT) {
        return "LEFT";
    }

    if (static_cast<int>(self) >= static_cast<int>(BorderPlace::UNKNOWN)) {
        return "UNKNOWN";
    }

    return "COMBO";

}

//---------------	WeightMap Methods	----------------------------
WeightMap::WeightMap(mapsize_t width, mapsize_t height) : width(width), height(height), arr(width, height) {
    if (width == 0 || height == 0) {
        throw std::invalid_argument("Zero passed as either width or height");
    }

    // Populate nodes
    for (fast_mapsize_t x = 0; x < width; x++) {
        for (fast_mapsize_t y = 0; y < height; y++) {
            new(&arr[x][y]) Node(x, y, std::numeric_limits<mapsize_t>::max(),
                                 std::numeric_limits<mapsize_t>::max(),
                                 this->getMinWeight(),
                                 std::numeric_limits<decltype(Node::g)>::infinity(),
                                 std::numeric_limits<decltype(Node::h)>::infinity());
        }
    }
}

weight_t *WeightMap::getWeights() const {
    weight_t *const buff = new weight_t[this->width * this->height];

    for (size_t y = 0; y < this->height; y++) {
        for (size_t x = 0; x < this->width; x++) {
            buff[(y * this->width) + x] = arr[x][y].weight;
        }
    }
    return buff;
}

bool WeightMap::operator==(const WeightMap &other) const {
    if (this->width != other.width || this->height != other.height) {
        return false;
    }
    for (size_t x = 0; x < this->width; x++) {
        for (size_t y = 0; y < this->height; y++) {
            if (this->arr[x][y].weight != other.arr[x][y].weight) {
                return false;
            }
        }
    }

    return true;
}

// Based off an implementation of Arrays.deepHashCode(e) from Java
size_t WeightMap::hash() const {
    constexpr
    const size_t P = 97;

    const std::hash <mapsize_t> map_hashf;
    const std::hash <weight_t> weight_hashf;

    size_t hash = map_hashf(this->width) + (map_hashf(this->height) * P);

    for (size_t x = 0; x < this->width; x++) {
        for (size_t y = 0; y < this->height; y++) {
            const size_t element_hash = weight_hashf(this->arr[x][y].weight);
            hash = (P * hash) + element_hash;
        }
    }
    return hash;
}

weight_t WeightMap::getMaxWeightInMap() const {
    weight_t curr_max = std::numeric_limits<weight_t>::min();

    for (fast_mapsize_t x = 0; x < this->width; x++) {
        for (fast_mapsize_t y = 0; y < this->height; y++) {
            curr_max = std::max(arr[x][y].weight, curr_max);
        }
    }
    return curr_max;
}

weight_t WeightMap::getWeight(mapsize_t x, mapsize_t y) const {
    if (!isValidPoint(x, y)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Point (%u, %u) out of bounds!", x, y);
        throw std::invalid_argument(std::string(error_message));
    }

    return arr[x][y].weight;
}

void WeightMap::setWeight(mapsize_t x, mapsize_t y, weight_t weight, bool overwrite) {
    if (!isValidPoint(x, y)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Point (%u, %u) out of bounds!", x, y);
        throw std::invalid_argument(std::string(error_message));
    }
    if (weight > this->getMaxWeight()) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Weight %u is greater than the max weight %u!", weight,
                      this->getMaxWeight());
        throw std::invalid_argument(std::string(error_message));
    }
    if (weight < this->getMinWeight()) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Weight %u is less than the minimum weight %u!", weight,
                      this->getMaxWeight());
        throw std::invalid_argument(std::string(error_message));
    }
    arr[x][y].weight = overwrite ? weight : std::max(arr[x][y].weight, weight);
}

std::string WeightMap::to_string(bool extraData) const {

    std::stringstream ss;

    if (extraData) {
        ss << *this;
    } else {

        const size_t max_width = std::to_string((int) this->getMaxWeightInMap()).size();

        for (fast_mapsize_t y = 0; y < height; y++) {
            for (fast_mapsize_t x = 0; x < width; x++) {
                ss << std::setw(max_width) << std::setfill(' ') << arr[x][y].weight << ' ';
            }
            ss << '\n';
        }
    }

    return ss.str();
}

WeightMap::path_t WeightMap::backtracePath(const Node &src, const Node &dst) const {
    path_t path;

    const Node *currentNode = &dst;
    while (isValidPoint(currentNode->parent_x, currentNode->parent_y)) {
        path.emplace_back(currentNode->x, currentNode->y);
        currentNode = &this->arr[currentNode->parent_x][currentNode->parent_y];
    }
    assert(path.size() <= (this->height * this->width));
    path.emplace_back(currentNode->x, currentNode->y);
    std::reverse(path.begin(), path.end());

// This code might be redundant
//    // Remove points that are redundant
//    if (path.size() < 3) {
//        assert(path[0].first == src.x && path[0].second == src.y);
//        return path;
//    }
//    for (int i = path.size() - 2; i > 0; i--) {
//        // Last point to current
//        int16_t pcx = path[i - 1].first - path[i].first;
//        int16_t pcy = path[i - 1].second - path[i].second;
//        // Current point to next
//        int16_t cnx = path[i].first + path[i + 1].first;
//        int16_t cny = path[i].second + path[i + 1].second;
//        // Previous point to next
//        int16_t pnx = path[i - 1].first - path[i + 1].first;
//        int16_t pny = path[i - 1].second - path[i + 1].second;
//
//        // See what the different between (l->c + c->n) and (p->n)
//        float dist_diff = ((std::sqrt(pcx * pcx + pcy * pcy) + std::sqrt(cnx * cnx + cny * cny)) /
//                           std::sqrt(pnx * pnx + pny * pny)) - 1.0f;
//
//        if (std::abs(dist_diff) < 1.0) {
//            path.erase(path.begin() + i);
//        }
//    }

    assert(path[0].first == src.x && path[0].second == src.y);
    return path;
}

WeightMap::path_t
WeightMap::getPath(mapsize_t srcX, mapsize_t srcY, mapsize_t dstX, mapsize_t dstY, weight_t turn_cost) {
    if (!isValidPoint(srcX, srcY)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Source Point (%u, %u) out of bounds!", srcX, srcY);
        throw std::invalid_argument(std::string(error_message));
    }
    if (!isValidPoint(dstX, dstY)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Destination Point (%u, %u) out of bounds!", dstX, dstY);
        throw std::invalid_argument(std::string(error_message));
    }

    if (srcX == dstX && srcY == dstY) {
        return {{srcX, srcY}};
    }

    Node &src = this->arr[srcX][srcY];
    Node &dst = this->arr[dstX][dstY];

    this->resetMap();

    // Set heuristic values for all points in map
    for (mapsize_t x = 0; x < width; x++) {
        for (mapsize_t y = 0; y < height; y++) {
            arr[x][y].h = distance(x, y, dstX, dstY);
        }
    }

    src.parent_x = std::numeric_limits<mapsize_t>::max();
    src.parent_y = std::numeric_limits<mapsize_t>::max();
    src.g = 0;

    std::vector < Node * > queue_backer;
    queue_backer.reserve(this->width * this->height);
    WeightMap::queue_t q(NodeCmp(), std::move(queue_backer));
    q.push(&src);
    while (!q.empty()) {
        Node &currentNode = *q.top();
        q.pop();

        if (currentNode.x == dst.x && currentNode.y == dst.y) {
            // Generate Path
            return backtracePath(src, dst);
        }

        thetaMakeMoves(moves, numMoves, currentNode, q, turn_cost);
    }

    // We should not reach here. The if statement in the while loop should have triggered for at least one node's coordinates
    assert(false);
}


WeightMap::path_t WeightMap::getPathToX(mapsize_t srcX, mapsize_t srcY, mapsize_t dstX, weight_t turn_cost) {
    if (!isValidPoint(srcX, srcY)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Source Point (%u, %u) out of bounds!", srcX, srcY);
        throw std::invalid_argument(std::string(error_message));
    }

    if (dstX >= this->width) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "X value to travel to {%u} out of bounds!", dstX);
        throw std::invalid_argument(std::string(error_message));
    }

    Node &src = this->arr[srcX][srcY];

    this->resetMap();

    // Set heuristic values for all points in map
    for (mapsize_t x = 0; x < width; x++) {
        weight_t dist_from_dstX = std::abs((int32_t) x - dstX);
        for (mapsize_t y = 0; y < height; y++) {
            arr[x][y].h = dist_from_dstX;
        }
    }

    src.parent_x = std::numeric_limits<mapsize_t>::max();
    src.parent_y = std::numeric_limits<mapsize_t>::max();
    src.g = 0;

    std::vector < Node * > queue_backer;
    queue_backer.reserve(
            static_cast<size_t>(this->width) * static_cast<size_t>(this->height));
    WeightMap::queue_t q(NodeCmp(), std::move(queue_backer));
    q.push(&src);
    while (!q.empty()) {
        // Taking a reference from a queue and destroying it is ok because the Node does not live in the queue,
        //  Rather, the queue holds references to a Node held by the weightmap, and we are copying out that reference
        //  And then removing the reference object from the queue
        Node &currentNode = *q.top();
        q.pop();

        if (dstX == currentNode.x) {
            return backtracePath(src, currentNode);
        }

        thetaMakeMoves(moves, numMoves, currentNode, q, turn_cost);

    }

    // We should not reach here. The if statement in the while loop should have triggered on the first node with the proper x-value
    assert(false);
}

void WeightMap::spreadDataArray(const weight_t data[], mapsize_t origin_x, mapsize_t origin_y, mapsize_t data_w, mapsize_t data_h, mapsize_t radius) {
    using namespace cv;

    Mat raw_data(data_h, data_w, CV_16U, &data);
    Mat padded_data(data_h + 2*radius, data_w + 2*radius, CV_16U);
    padded_data.setTo(Scalar::all(0));

    raw_data.copyTo(padded_data(Rect(radius, radius, raw_data.cols, raw_data.rows)));

    // Clamp the inputted values within the possible range (even if it's already clamped for some reason)
    padded_data = max(min(padded_data, Scalar::all(getMaxWeight())), Scalar::all(0));
    Mat dilated = padded_data.clone();
    float radius_f = radius;
    Mat temp(padded_data.rows, padded_data.cols, CV_16U);
    for (mapsize_t r = 1; r < radius; r++) {
        mapsize_t kernel_diameter = 2*r + 1;
        dilate(padded_data, temp,
               getStructuringElement(MORPH_ELLIPSE, Size(kernel_diameter, kernel_diameter)),
               Point(-1, -1), 1, BORDER_CONSTANT, 0);
        temp *= 1.0f - (float)r / radius_f;
        dilated = max(dilated, temp);
    }

    for (int x = 0; x < dilated.cols; x++) {
        int map_x = x-radius + origin_x;
        if (map_x < 0 || map_x >= width)
            continue;

        for (int y = 0; y < dilated.rows; y++) {
            int map_y = y-radius + origin_y;
            if (map_y < 0 || map_y >= height)
                continue;
            int val = dilated.at<weight_t>(x, y);
//            int val = padded_data.at<weight_t>(x, y);

            if (isValidWeight(val) && val > arr[map_x][map_y].weight)
                arr[map_x][map_y].weight = val;
        }
    }
}

void WeightMap::addBorder(mapsize_t border_width, weight_t border_weight, BorderPlace place, bool gradient, bool overwrite) {
    if (!WeightMap::isValidWeight(border_weight)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Weight {%u} out of bounds!", border_weight);
        throw std::invalid_argument(std::string(error_message));
    }

    if (!isValid(place)) {

        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Place {0x%x} is invalid!", static_cast<int>(place));
        throw std::invalid_argument(std::string(error_message));
    }

    if ((contains(place, BorderPlace::TOP) || contains(place, BorderPlace::BOTTOM)) && border_width > height) {
        throw std::invalid_argument("Border Width is greater than the height of the board");
    }

    if ((contains(place, BorderPlace::RIGHT) || contains(place, BorderPlace::LEFT)) && border_width > width) {
        throw std::invalid_argument("Border Width is greater than the width of the board");
    }

    if (contains(place, BorderPlace::TOP) || contains(place, BorderPlace::BOTTOM)) {
        for (fast_mapsize_t y = 0; y < border_width; y++) {
            weight_t new_weight = gradient ? border_weight - (uint32_t)(y * border_weight) / border_width
                                           : border_weight;
            new_weight = std::min(getMaxWeight(), std::max(getMinWeight(), new_weight));
            for (fast_mapsize_t x = 0; x < width; x++) {
                if (contains(place, BorderPlace::TOP)) {
                    setWeight(x, y, new_weight, overwrite);
                }
                if (contains(place, BorderPlace::BOTTOM)) {
                    setWeight(x, height - y - 1, new_weight, overwrite);
                }
            }
        }
    }

    if (contains(place, BorderPlace::LEFT) || contains(place, BorderPlace::RIGHT)) {
        for (fast_mapsize_t x = 0; x < border_width; x++) {
            weight_t new_weight = gradient ? border_weight - (uint32_t)(x * border_weight) / border_width
                                           : border_weight;
            new_weight = std::min(getMaxWeight(), std::max(getMinWeight(), new_weight));
            for (fast_mapsize_t y = 0; y < height; y++) {
                if (contains(place, BorderPlace::LEFT)) {
                    setWeight(x, y, new_weight, overwrite);
                }
                if (contains(place, BorderPlace::RIGHT)) {
                    setWeight(width - x - 1, y, new_weight, overwrite);
                }
            }
        }
    }
}

void WeightMap::addCircle(mapsize_t x_in, mapsize_t y_in, mapsize_t radius, weight_t weight, bool gradient, bool overwrite) {
    if (!WeightMap::isValidWeight(weight)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Weight {%u} out of bounds!", weight);
        throw std::invalid_argument(std::string(error_message));
    }
    if (!isValidPoint(x_in, y_in)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Point (%u, %u) out of bounds!", x_in, y_in);
        throw std::invalid_argument(std::string(error_message));
    }

    setWeight(x_in, y_in, weight, overwrite); // Center Location

    // Axes points (dx=0 & dy=0)
    for (mapsize_t dist = 0; dist <= radius; dist++) {
        weight_t new_weight = gradient ? weight - (uint32_t)(weight * dist) / radius : weight;
        new_weight = std::min(getMaxWeight(), std::max(getMinWeight(), new_weight));

        //Calculate points
        using point = std::pair<size_t, size_t>;

        point points[4] = {
                {x_in,        y_in + dist},
                {x_in,        y_in - dist},
                {x_in + dist, y_in},
                {x_in - dist, y_in},
        };

        for (const auto &pt: points) {
            if (isValidPoint(pt.first, pt.second)) {
                setWeight(pt.first, pt.second, new_weight, overwrite);
            }
        }
    }

    // All other points in circle
    const fweight_t radius_f = radius; // Don't want to convert it to a float each time
    for (size_t dx = 1; dx <= radius; dx++) {
        for (size_t dy = 1; dy <= dx; dy++) {
            const fweight_t dist = std::sqrt((dx * dx) + (dy * dy));

            if (dist <= radius_f) {
                weight_t new_weight = gradient ? (1.0 - (dist / radius_f)) * weight : weight;
                new_weight = std::min(getMaxWeight(), std::max(getMinWeight(), new_weight));

                // Calculate points
                using point = std::pair<size_t, size_t>;

                // Exploiting 8-way symmetry to avoid repeated distance calculations
                point points[8] = {
                        {x_in + dx, y_in + dy}, // Standard Location
                        {x_in - dx, y_in + dy}, // Symmetry across x-axis
                        {x_in + dx, y_in - dy}, // Symmetry across y-axis
                        {x_in - dx, y_in - dy}, // Symmetry across x = -y

                        //These points are the above, but reflected across y=x
                        {x_in + dy, y_in + dx}, // Standard Location
                        {x_in - dy, y_in + dx}, // Symmetry across x-axis
                        {x_in + dy, y_in - dx}, // Symmetry across y-axis
                        {x_in - dy, y_in - dx}, // Symmetry across x = -y
                };

                for (const auto &pt: points) {
                    if (isValidPoint(pt.first, pt.second)) {
                        setWeight(pt.first, pt.second, new_weight, overwrite);
                    }
                }
            }
        }
    }
}

void WeightMap::addRectangle(mapsize_t x_in, mapsize_t y_in, mapsize_t w_in, mapsize_t h_in, weight_t weight,
                             mapsize_t radius, bool overwrite) {
    if (!WeightMap::isValidWeight(weight)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Weight {%u} out of bounds!", weight);
        throw std::invalid_argument(std::string(error_message));
    }
    if (!isValidPoint(x_in, y_in)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Corner point (%u, %u) out of bounds!", x_in, y_in);
        throw std::invalid_argument(std::string(error_message));
    }
    if (!isValidPoint(x_in + w_in, y_in + h_in)) {
        char error_message[60];
        std::snprintf(error_message, sizeof(error_message), "Corner point (%u, %u) out of bounds!", x_in + w_in,
                      y_in + h_in);
        throw std::invalid_argument(std::string(error_message));
    }

    // Inner max weights
    for (mapsize_t x = x_in; x < x_in + w_in; x++) {
        for (mapsize_t y = y_in; y < y_in + h_in; y++) {
            setWeight(x, y, weight, overwrite);
        }
    }

    if (radius > 0) {
        // Spread along rect side
        for (mapsize_t dist = 0; dist <= radius; dist++) {
            weight_t new_weight = weight - (uint32_t)(weight * dist) / radius;
            new_weight = std::min(getMaxWeight(), std::max(getMinWeight(), new_weight));

            for (mapsize_t x = x_in; x <= x_in + w_in; x++) {
                if (isValidPoint(x, y_in - dist)) {
                    setWeight(x, y_in - dist, new_weight, overwrite);
                }
                if (isValidPoint(x, y_in + h_in + dist)) {
                    setWeight(x, y_in + h_in + dist, new_weight, overwrite);
                }
            }
            for (mapsize_t y = y_in; y <= y_in + h_in; y++) {
                if (isValidPoint(x_in - dist, y)) {
                    setWeight(x_in - dist, y, new_weight, overwrite);
                }
                if (isValidPoint(x_in + w_in + dist, y)) {
                    setWeight(x_in + w_in + dist, y, new_weight, overwrite);
                }
            }
        }

        // Corner spread
        const fweight_t radius_f = radius; // Don't want to convert it to a float each time
        for (size_t dx = 1; dx <= radius; dx++) {
            for (size_t dy = 1; dy <= dx; dy++) {
                const fweight_t dist = std::sqrt((dx * dx) + (dy * dy));

                if (dist <= radius_f) {
                    weight_t new_weight = (1.0 - (dist / radius_f)) * weight;
                    new_weight = std::min(getMaxWeight(), std::max(getMinWeight(), new_weight));

                    // Calculate points
                    using point = std::pair<size_t, size_t>;

                    point points[8] = {
                            {x_in + w_in + dx, y_in + h_in + dy}, // Standard Location
                            {x_in - dx,        y_in + h_in + dy}, // Symmetry across x-axis
                            {x_in + w_in + dx, y_in - dy}, // Symmetry across y-axis
                            {x_in - dx,        y_in - dy}, // Symmetry across x = -y

                            //These points are the above, but reflected across y=x
                            {x_in + w_in + dy, y_in + h_in + dx}, // Standard Location
                            {x_in - dy,        y_in + h_in + dx}, // Symmetry across x-axis
                            {x_in + w_in + dy, y_in - dx}, // Symmetry across y-axis
                            {x_in - dy,        y_in - dx}, // Symmetry across x = -y
                    };

                    for (const auto &pt: points) {
                        if (isValidPoint(pt.first, pt.second)) {
                            setWeight(pt.first, pt.second, new_weight, overwrite);
                        }
                    }
                }
            }
        }
    }
}

bool WeightMap::isValidPoint(int32_t x, int32_t y) const {
    return (x < this->width && y < this->height) && (x >= 0 && y >= 0);
}

std::string WeightMap::path_to_str(path_t &path) {
    std::stringstream ss;
    for (auto &point: path) {
        ss << '(' << point.first << ", " << point.second << ")\n";
    }
    return ss.str();
}

std::string WeightMap::point_to_string(const point_t &pt) {
    std::stringstream ss;
    ss << '(' << pt.first << ", " << pt.second << ')';
    return ss.str();
}

// Helper Functions
void WeightMap::resetMap() {
    for (fast_mapsize_t x = 0; x < width; x++) {
        for (fast_mapsize_t y = 0; y < height; y++) {
            // Set up default values
            Node &current_node = arr[x][y];
            current_node.parent_x = std::numeric_limits<mapsize_t>::max();
            current_node.parent_y = std::numeric_limits<mapsize_t>::max();
            current_node.g = std::numeric_limits<decltype(current_node.g)>::infinity();
        }
    }
}

void WeightMap::thetaMakeMoves(const NeighborsMove *moves, const size_t numMoves, const Node &currentNode,
                               WeightMap::queue_t &q, weight_t extra_cost_for_turn) {
    for (size_t i = 0; i < numMoves; i++) {
        const int32_t xidx = currentNode.x + moves[i].dx;
        const int32_t yidx = currentNode.y + moves[i].dy;

        // Bounds Check
        if (!isValidPoint(xidx, yidx)) {
            continue;
        }
        Node &neighborNode = arr[(size_t) xidx][(size_t) yidx];

        fweight_t pivotCost = extra_cost_for_turn + currentNode.g +
                              moves[i].weight_multiplier * ((neighborNode.weight + currentNode.weight) / 2.0);
        if (pivotCost < neighborNode.g) {
            if (isValidPoint(currentNode.parent_x, currentNode.parent_y)) { // if currentNode has neighbor
                Node &currentParentNode = arr[currentNode.parent_x][currentNode.parent_y];
                fweight_t directCost = currentParentNode.g + get_linear_cost(currentParentNode, neighborNode);
                if (directCost <= pivotCost) {
                    neighborNode.parent_x = currentParentNode.x;
                    neighborNode.parent_y = currentParentNode.y;
                    neighborNode.g = directCost;
                    q.push(&neighborNode);
                    continue;
                }
            }
            neighborNode.parent_x = currentNode.x;
            neighborNode.parent_y = currentNode.y;
            neighborNode.g = pivotCost;
            q.push(&neighborNode);
        }
    }
}

// First code: modified DDA I think? (uses floating points, more accurate result)
// Second code: Bresenham's line algo (uses integer math, less accurate result)

// On my system, implementation 1 is faster, but we might want to test on hardware to see which is faster

fweight_t WeightMap::get_linear_cost(Node &a, Node &b) {
#if 1
    fweight_t sum = 0.0;

    if (a.x == b.x && a.y == b.y) {
        return arr[a.x][a.y].weight;
    }

    if (a.x == b.x) { // vertical line
        sum += (arr[a.x][a.y].weight + arr[b.x][b.y].weight) / 2.0f; // Add the endpoints of the line
        for (mapsize_t y = std::min(a.y, b.y) + 1; y < std::max(a.y, b.y); y++) {
            sum += arr[a.x][y].weight;
        }
        return sum;
    }
    if (a.y == b.y) {// horizontal line
        sum += (arr[a.x][a.y].weight + arr[b.x][b.y].weight) / 2.0f; // Add the endpoints of the line
        for (mapsize_t x = std::min(a.x, b.x) + 1; x < std::max(a.x, b.x); x++) {
            sum += arr[x][a.y].weight;
        }
        return sum;
    }
    if (b.x - a.x == b.y - a.y || b.x - a.x == a.y - b.y) { // line of slope 1 or -1
        int8_t sm = b.x - a.x == b.y - a.y ? 1 : -1; // Sign of slope (magnitude is sqrt2
        sum += (arr[a.x][a.y].weight + arr[b.x][b.y].weight) / 2.0f; // Add the endpoints of the line
        const mapsize_t x1 = std::min(a.x, b.x);
        mapsize_t y1 = x1 == a.x ? a.y : b.y;
        for (mapsize_t i = 1; i < std::abs((int32_t) b.x - a.x); i++) {
            sum += arr[x1 + i][y1 + sm * i].weight;
        }
        return sum * 1.42f; // Convert 1D line weight to 2D distance by multiplying by sqrt2 (sqrt(1+m*m) where m=1)
    }

    float m = (float) (b.y - a.y) / (float) (b.x - a.x);
    const int8_t sm = m > 0 ? 1 : -1;
    m *= sm;

    if (m < 1.0) {  // dy < dx, so incrementing x by 1 safely included all y values
        const mapsize_t x1 = std::min(a.x, b.x);
        const mapsize_t x2 = std::max(a.x, b.x);
        mapsize_t y = (x1 == a.x) ? a.y : b.y;
        sum += arr[x1][y].weight / 2.0f;

        float err = m / 2.0f;
        for (mapsize_t x = x1 + 1; x < x2; x++) {
            if (err + m >= 0.5f) { // The line crosses into new vertical tile
                const float t = (0.5f - err) / m; // How far across x before it crosses
                sum += arr[x][y].weight * t;
                y += sm;
                err -= 1.0;
                sum += arr[x][y].weight * (1.0f - t);
            } else {
                sum += arr[x][y].weight;
            }
            err += m;
        }
        sum += arr[x2][y].weight / 2.0f;
    } else {  // dy > dx, so incrementing y by 1 safely included all x values
        m = 1.0f / m; // Flip x and y, so take reciprocal of slope to adjust (dy/dx) => (dx/dy)

        const mapsize_t y1 = std::min(a.y, b.y);
        const mapsize_t y2 = std::max(a.y, b.y);
        mapsize_t x = (y1 == a.y) ? a.x : b.x;
        sum += arr[x][y1].weight / 2.0f; // Add half the weight of the starting node

        float err = m / 2;
        for (mapsize_t y = y1 + 1; y < y2; y++) {
            if (err + m >= 0.5f) { // The line crosses into new horizontal tile
                const float t = (0.5f - err) / m; // How far across y before it crosses
                sum += arr[x][y].weight * t;
                x += sm;
                err -= 1.0;
                sum += arr[x][y].weight * (1.0f - t);
            } else {
                sum += arr[x][y].weight;
            }
            err += m;
        }
        sum += arr[x][y2].weight / 2.0f; // Add half the weight of the ending node
    }
    // The sum is only accounting for the length of the line of its largests axis,
    // so we multiply it by sqrt(1 + m^2) to account for the length of the whole line
    return sum * std::sqrt(1 + m * m);

#else
    const int32_t dx = std::abs((int32_t)b.x - a.x);
    const int32_t dy = std::abs((int32_t)b.y - a.y);
    const int8_t sx = (a.x < b.x) ? 1 : -1;
    const int8_t sy = (a.y < b.y) ? 1 : -1;
    int32_t err = dx - dy;

    fast_mapsize_t x = a.x;
    fast_mapsize_t y = a.y;

    weight_t adjacent_moves_cost = 0;
    weight_t diagonal_moves_cost = 0;

    while(true) {
        fast_mapsize_t px = x;
        fast_mapsize_t py = y;
        if (x == b.x && y == b.y) {
            return adjacent_moves_cost + 1.42f * diagonal_moves_cost;
        }
        int32_t e2 = err * 2;
        uint8_t diag = 0;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
            diag++;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
            diag++;
        }
        if (diag >= 2) { // Add them up as integers for speed over float operations
            diagonal_moves_cost += (arr[x][y].weight + arr[px][py].weight) / 2;
        } else {
            adjacent_moves_cost += (arr[x][y].weight + arr[px][py].weight) / 2;
        }
    }
#endif
}

fweight_t WeightMap::distance(fweight_t x1, fweight_t y1, fweight_t x2, fweight_t y2) {
    return (fweight_t) std::sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
}


//Weight Map Serialization / Deserialization

#if defined(_MSC_VER) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#include <winsock.h>
#elif defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#else
#   error "Unknown compiler"
#endif


std::pair<const char *, const size_t> WeightMap::serialize() const {
    static_assert((sizeof(mapsize_t) == sizeof(weight_t)) && (sizeof(weight_t) == 2),
                  "Current Serialization assumes 2 byte mapsize_t and weight_t due to htonl");
    const size_t buff_size_bytes = (sizeof(mapsize_t) * 2) + (sizeof(weight_t) * this->width * this->height);

    char *const buff = (char *) malloc(buff_size_bytes);
    if (buff == nullptr) {
        throw std::bad_alloc();
    }

    reinterpret_cast<mapsize_t *>(buff)[0] = htons(this->width);
    reinterpret_cast<mapsize_t *>(buff)[1] = htons(this->height);

    weight_t *const weight_buff = (weight_t * ) & (((mapsize_t *) buff)[2]);
    for (size_t y = 0; y < this->height; y++) {
        for (size_t x = 0; x < this->width; x++) {
            weight_buff[(y * this->width) + x] = htons(arr[x][y].weight);
        }
    }

    return std::make_pair(buff, buff_size_bytes);
}

WeightMap WeightMap::deserialize(std::pair<const char *, const size_t> bytes) {
    if (bytes.second <= 4) {
        throw std::invalid_argument("Not enough bytes to construct a WeightMap");
    }

    const char *const buff = bytes.first;

    const mapsize_t *const map_buff = (const mapsize_t *) buff;

    const mapsize_t width = ntohs(map_buff[0]);
    const mapsize_t height = ntohs(map_buff[1]);

    const size_t expected_buffer_size = (sizeof(mapsize_t) * 2) + (sizeof(weight_t) * width * height);
    if (bytes.second < expected_buffer_size) {
        throw std::invalid_argument("Not enough bytes to construct a WeightMap of given size");
    }

    WeightMap wm(width, height);

    const weight_t *const weight_buff = (const weight_t *) &(map_buff[2]);
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            wm.arr[x][y].weight = ntohs(weight_buff[(y * width) + x]);
        }
    }

    return wm;
}

//---------------	Node Methods	----------------------------
bool WeightMap::Node::operator==(const WeightMap::Node &other) {
    auto b = other;
    auto a = *this;
    return (
            a.x == b.x &&
            a.y == b.y &&
            a.weight == b.weight &&
            a.parent_x == b.parent_x &&
            a.parent_y == b.parent_y &&
            a.g == b.g &&
            a.h == b.h);
}

std::ostream &operator<<(std::ostream &os, const WeightMap::Node &n) {
    os << "{ X: " << n.x << "; Y: " << n.y << "; Weight: " << n.weight << "; Parent: (" << n.parent_x << ','
       << n.parent_y << "); g: " << n.g << "; h: " << n.h << "}";
    return os;
}

std::ostream &operator<<(std::ostream &os, const WeightMap &wm) {
    for (size_t y = 0; y < wm.height; y++) {
        for (size_t x = 0; x < wm.width; x++) {
            os << wm.arr[x][y] << ' ';
        }
        os << '\n';
    }

    return os;
}

bool WeightMap::NodeCmp::operator()(const Node &a, const Node &b) {
    return a.g + a.h > b.g + b.h;
}

bool WeightMap::NodeCmp::operator()(const Node *a, const Node *b) {
    return a->g + a->h > b->g + b->h;
}
