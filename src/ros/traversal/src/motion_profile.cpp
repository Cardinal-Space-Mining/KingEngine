#include "motion_profile.hpp"

#include <limits>	 //std::numeric_limits
#include <algorithm> //std::min
#include <stdexcept> //std::out_of_range, std::invalid_argument
#include <sstream>	 //std::stringstream
#include <iomanip>	 //std::setw
#include <algorithm> //std::reverse
#include <cmath>	 //std::sqrt
#include <array>
#include <functional>
#include <vector>
#include <cmath>
#include <memory>
#include <iostream>
#include <stdlib.h>  

using point = std::pair<uint16_t, uint16_t>;

using namespace std;


//------------Motion Profile-------------
profile::profile() {
	linear_velocity = 0;
	angular_velocity = 0;
	heading = 0;

};

//Compile the angular and linear velocity of the profile to form percentages for each tracks
//Should return a pair of integers
std::pair<double, double> profile::get_setspeed() {
	return 0;
};




//follow path
/*
	Take the given node vector, consider it only one node at a time?
	Find the point on the path where the current position is closest to
	If the distance away from the path is great enough, shit your pants
	Get the tangent angle at that point. Get the current angle to adjust velocity to the target angle.
				If the current angle is sufficiently different, then do a point turn
	If the distance away from the target point is small enough, then switch to the next target
	Looping
*/

void profile::follow_path() {
	using path_t = std::vector<point>;

	using motion_path = std::vector<motion_node>;

	motion_path curpath = motion_path.front();

	//Get update all local variables

	point closest_point = curpath.get_closest_point(this->current);
    //if the closest point is beyond the boundaries, then move to the next spot on the path;
    //Perhaps we can decide that the boundary is a radius around the point



	//if the closest point is the target 

	this->tar_angle = curpath.get_tangent_angle(closest_point);
    

	//compare tar angle to current angle, then adjust the linear & angular velocity based on the difference
    double velocity = abs(cur_angle - tar_angle);

    this->linear_velocity = 0.8;
    this->angular_velocity = 0.0;

	//send the 'get_velocity' function to wherever sends the velocity


	


}



//-------------Motion Structs-------------

void profile::compile_path_linear(std::vector<point_t> path) {

    if (path.size() < 2) {
        std::cerr << "Path must contain at least 2 points for compilation." << std::endl;
        return;
    }

    motion_path.clear();

    for (size_t i = 0; i < path.size() - 1; ++i) {
        motion_path.push_back(std::make_shared<linear_motion>(path[i], path[i + 1]));
    }


}


void profile::compile_path_hypocycloid(std::vector<point_t> path) {
    if (path.size() < 2) {
        std::cerr << "Path must contain at least 2 points for compilation." << std::endl;
        return;
    }

    motion_path.clear();

    // Generate linear motion path
    for (size_t i = 0; i < path.size() - 1; ++i) {
        motion_path.push_back(std::make_shared<linear_motion>(path[i], path[i + 1]));
    }

    // Calculate and insert hypocycloidal motion nodes between linear motion nodes
    for (size_t i = 0; i < motion_path.size() - 1; ++i) {
        auto linear_node_a = std::dynamic_pointer_cast<linear_motion>(motion_path[i]);
        auto linear_node_b = std::dynamic_pointer_cast<linear_motion>(motion_path[i + 1]);
        if (linear_node_a && linear_node_b) {
            double radius = 0.5 * std::sqrt(std::pow(path[i + 1].x - path[i].x, 2) + std::pow(path[i + 1].y - path[i].y, 2));
            motion_path.insert(motion_path.begin() + i + 1, std::make_shared<hypocycloid_motion>(path[i], path[i + 1], radius));
        }
    }

    // Recompile the path to ensure smooth transition between nodes
    std::vector<point> compiled_path;
    for (const auto& node : motion_path) {
        compiled_path.push_back(node->a);
    }
    compiled_path.push_back(motion_path.back()->b);


}

void profile::compile_path_bezier(std::vector<point_t> path) {
    if (path.size() < 2) {
        std::cerr << "Path must contain at least 2 points for compilation." << std::endl;
        return;
    }

    motion_path.clear();

    // Generate linear motion path
    for (size_t i = 0; i < path.size() - 1; ++i) {
        motion_path.push_back(std::make_shared<linear_motion>(path[i], path[i + 1]));
    }

    // Calculate and insert Bezier curve motion nodes between linear motion nodes
    for (size_t i = 0; i < motion_path.size() - 1; ++i) {
        auto linear_node_a = std::dynamic_pointer_cast<linear_motion>(motion_path[i]);
        auto linear_node_b = std::dynamic_pointer_cast<linear_motion>(motion_path[i + 1]);
        if (linear_node_a && linear_node_b) {
            // Calculate control points for Bezier curve
            point p0 = linear_node_a->get_closest_point(path[i]);
            point p3 = linear_node_b->get_closest_point(path[i + 1]);
            point p1 = {
                p0.x + (p3.x - p0.x) / 3.0,
                p0.y + (p3.y - p0.y) / 3.0
            };
            point p2 = {
                p3.x - (p3.x - p0.x) / 3.0,
                p3.y - (p3.y - p0.y) / 3.0
            };
            motion_path.insert(motion_path.begin() + i + 1, std::make_shared<bezier_motion>(p0, p1, p2, p3));
        }
    }
}





//-------------Linear Motion--------------

double linear_motion::get_tangent_angle(point p) {
    return atan2(b.y - a.y, b.x - a.x);
}

point linear_motion::get_closest_point(const point& current) override {
    double x_diff = b.x - a.x;
    double y_diff = b.y - a.y;
    double t = ((current.x - a.x) * x_diff + (current.y - a.y) * y_diff) / (x_diff * x_diff + y_diff * y_diff);
    if (t < 0) {
        return a;
    }
    else if (t > 1) {
        return b;
    }
    else {
        return point{a.x + t * x_diff, a.y + t * y_diff};
    }
}


//-------------Bezier Motion--------------
double bezier_motion::get_tangent_angle(const point& p) const override {
    // For simplicity, we'll use the tangent angle at the start point of the Bezier curve
    return atan2(p1.y - p0.y, p1.x - p0.x);
}

point bezier_motion::get_closest_point(const point& current) const override {
    // Not implemented for Bezier curves
    return p0; // Placeholder
}


//-------------Cycloid Motion-------------



double hypocycloid_motion::get_tangent_angle(const point& p)  {
    double angle = atan2(b.y - a.y, b.x - a.x);
    double theta = atan2(p.y - a.y, p.x - a.x);
    double diff = angle - theta;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return angle + diff / 2;
}

point hypocycloid_motion::get_closest_point(const point& current) {
    double distance = sqrt((current.x - a.x) * (current.x - a.x) + (current.y - a.y) * (current.y - a.y));
    double theta = atan2(b.y - a.y, b.x - a.x);
    double t = distance / radius;
    double x = a.x + radius * cos(theta + t);
    double y = a.y + radius * sin(theta + t);
    return point{x, y};
}

