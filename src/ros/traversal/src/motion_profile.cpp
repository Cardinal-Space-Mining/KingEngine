#include "motion_profile.hpp"
#define _USE_MATH_DEFINES
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
#include <cstdlib>

//using point = std::pair<uint16_t, uint16_t>;

using namespace std;

//------------Utility Functions----------
double distance(point A, point B) {
    return sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2));
}

//Point A is the current point,
//Point B is the point along the line segment
//Hypotenuse is the length of the 'stick'
//Prefer left dictates whether or not the point C should be left or right of the line segment AB
point findCarrot(point A, point B, double hypotenuse, bool prefer_left = false) {
    // Calculate the distance between points A and B
    double base_length = distance(A, B);

    // Calculate the height of the right triangle (distance between point A and the line defined by B)
    double height = sqrt(pow(hypotenuse, 2) - pow(base_length, 2));

    // Calculate the direction from A to B
    double dir_x = (B.x - A.x) / base_length;
    double dir_y = (B.y - A.y) / base_length;

    // Determine the perpendicular direction
    double perp_dir_x = -dir_y; // Swap and negate one component to obtain a perpendicular vector
    double perp_dir_y = dir_x;

    // Determine the sign of the perpendicular direction based on the desired criteria
    if (!prefer_left) {
        perp_dir_x = -perp_dir_x;
        perp_dir_y = -perp_dir_y;
    }

    // Calculate the coordinates of point C by moving along the perpendicular direction
    point C;
    C.x = B.x + height * perp_dir_x; // Perpendicular direction
    C.y = B.y + height * perp_dir_y; // Perpendicular direction

    return C;
}

//Use this function when the current point and the closest point to the curve are the same point
//points A and B form a line segment
//point C is the current point (
point findCarrotSamePoint(point A, point B, point C, double distance_CD, bool prefer_left = true) {
    // Calculate the vector AB
    double vec_AB_x = B.x - A.x;
    double vec_AB_y = B.y - A.y;

    // Calculate the length of vector AB
    double length_AB = distance(A, B);

    // Normalize the vector AB to get the unit vector
    double unit_AB_x = vec_AB_x / length_AB;
    double unit_AB_y = vec_AB_y / length_AB;

    // Determine the sign of the direction based on the desired criteria
    double direction = prefer_left ? 1 : -1;

    // Scale the unit vector AB by the distance CD to get the vector from C to D
    double vec_CD_x = unit_AB_x * direction * distance_CD;
    double vec_CD_y = unit_AB_y * direction * distance_CD;

    // Calculate the coordinates of point D by adding the vector CD to point C
    point D;
    D.x = C.x + vec_CD_x;
    D.y = C.y + vec_CD_y;

    // Check if point D lies outside the segment AB
    if ((D.x - A.x) * (D.x - B.x) > 0 || (D.y - A.y) * (D.y - B.y) > 0) {
        // If D is outside the segment, set it to the closest endpoint of the segment
        double dist_to_A = distance(C, A);
        double dist_to_B = distance(C, B);
        if (dist_to_A < dist_to_B) {
            D = A;
        }
        else {
            D = B;
        }
    }

    return D;
}

//------------Motion Profile-------------

//Compile the angular and linear velocity of the profile to form percentages for each tracks
//Should return a pair of integers
std::pair<double, double> profile::get_speed() {
	return p(0.0, 0.0);
};




//-------------Profile methods-------------

void profile::compile_path_linear(std::vector<point> path) {
    if (this->path == nullptr) return;

    if (path.size() < 2) {
        std::cerr << "Path must contain at least 2 points for compilation." << std::endl;
        return;
    }

    this->motion_path.clear();

    for (size_t i = 0; i < path.size() - 1; ++i) {
        this->motion_path->push_back(std::make_shared<linear_motion>(path[i], path[i + 1]));
    }

    return;

}

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
    if (this.path == nullptr) return;
    if (this.m_path == nullptr) return;

	motion_node curnode = m_path->front();

    //Get the carrot point

    point target = curnode.get_target_from_distance(current, distance); //given the distance get the target
    
    //compare the current point to the target point
    //If the front of our path vector is also the end, then we don't want to do all this stuff because the last point is our final target
    if (curnode.get_end() == target && m_path->front() != m_path.->end()) {
        //Erase the first path from m_path
        m_path->erase(m_path->front());

        curnode = m_path->front();

        target = curnode.get_target_from_distance(current, distance);
    }
    
    //Assumes a normalized target angle
	//if the closest point is the target 
    this->cur_angle = setHeading(target);

    double headings[2] = { (tar_angle - 360 - cur_angle), (tar_angle + 360 - cur_angle) };

    double shortestDiff = abs(tar_angle - current);

    for (int i = 0; i < 2; i++) {
       double t = abs(headings[i]);
        if (t < shortestDiff) shortestDiff = t;
    }

    //Get the actual distance away from the target. Useful for final approach, the 'stick' length is broken
    
    double actual_distance = distance(this->current, target);
	//compare shortest target angle to the current angle, then adjust the linear & angular velocity based on the difference
    //If the difference in degrees is greater than 30, then do a point turn (linear velocity is zero)
    
    double percentage = abs(shortestDiff / 360);
    if (shortestDiff < 0) percentage = percentage * -1;

    if (!at_destination) {
        if (shortestDiff > 30) {
                this->linear_velocity = 0.0;
                this->angular_velocity = percentage;
            }
            else {
                this->linear_velocity = (1 - percentage); //If we are at our current heading, go 100%
                this->angular_velocity = percentage;
            }
    }
    else {
        //If we are finished, then this method should not be setting the velocities to anything.
        linear_velocity = 0.0;
        angular_velocity = 0.0;
    }
    
   
    return;
}


void profile::setHeading(point target) {
    // Calculate the differences in x and y coordinates
    double dx = target.x - current.x;
    double dy = target.y - current.y;

    // Calculate the angle (in radians) using arctan2
    double angle_rad = std::atan2(dy, dx);

    // Convert the angle from radians to degrees
    double angle_deg = angle_rad * 180 / M_PI;


    this->setTargetAngle(angle_deg);
}



void profile::setTargetAngle(double target)
{
    double normal = target % 360;
    if (normal < 0) {
        normal += 360;
    }

    tar_angle = normal;
}



void profile::pointTurn() {
    double headings[2] = { (final_angle - 360 - cur_angle), (final_angle + 360 - cur_angle) };

    double shortestDiff = abs(final_angle - current);

    for (int i = 0; i < 2; i++) {
        double t = abs(headings[i]);
        if (t < shortestDiff) shortestDiff = t;
    }

    double percentage = abs(shortestDiff / 360);
    if (shortestDiff < 0) percentage = percentage * -1;

    this->linear_velocity = 0.0;
    this->angular_velocity = percentage * ;
}





//-------------Linear Motion--------------

double linear_motion::get_tangent_angle(point p) {
    return atan2(b.y - a.y, b.x - a.x);
}


point linear_motion::get_closest_point(const point& current) override {
    double x_diff = b.x - a.x;
    double y_diff = b.y - a.y;
    double t = ((current.x - a.x) * x_diff + (current.y - a.y) * y_diff) / (x_diff * x_diff + y_diff * y_diff);
   /* if (t < 0) {
        return a;
    }*/
    if (t > 1) {
        return b;
    }
    else {
        return point{a.x + t * x_diff, a.y + t * y_diff};
    }
}

//Gets the target point that the carrot should point to. 
//When interpreting this information, if the returned point is the same as point 'b' from this segment,
//     assume that the distance does not hold then Move to the next segment
point linear_motion::get_target_from_distance(point current, double distance) {
    point closest = get_closest_point(current);
    
    point c = { 0, 0 };

    //Overloaded operator.
    if (current == closest) {
        c = findCarrotSamePoint(this.a, this.b, closest, distance, true);
    }
    else {
        c = findCarrot(current, b, distance);
    }

    return c;
}
