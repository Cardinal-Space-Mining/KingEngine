#pragma once

#include <ostream>	  //std::ostream
#include <stdint.h>	  //uint16_t, uint_fast16_t
#include <string>	  //std::string
#include <vector>	  //std::vector
#include <utility>	  //std::pair
#include <queue>	  //std::queue
#include <functional> //std::reference_wrapper

// Forward declaration
class motion_node;

struct point
{
	double x, y;

	bool operator==(const point &b) const
	{
		return ((x > b.x - 0.1 && x < b.x + 0.1) && (y > b.y - 0.1 && y < b.y + 0.1));
	}

	point(double x, double y) : x(x), y(y){};

	bool close(const point &b, double epsilon) {
		return ((x > b.x - epsilon && x < b.x + epsilon) && (y > b.y - epsilon && y < b.y + epsilon));
	};

	point() = default;
};

class motion_node
{
public:
	// double get_tangent_angle(point p);
	point get_closest_point(point current);
	point get_target_from_distance(point current, double distance);

	point get_start() { return a; }
	point get_end() { return b; }

	bool operator==(const motion_node &other) const
	{
		return a == other.a && b == other.b;
	}

	motion_node(const point &a, const point &b) : a(a), b(b) {}
	motion_node() {}
	~motion_node() = default;

private:
	point a;
	point b;
};

// class linear_motion : public motion_node {
// public:
// 	linear_motion(point a, point b) : motion_node(a, b) {}
// 	linear_motion() = default;
// };

class profile
{
public:
	// profile(std::vector<point> p) : path(p)
	// {
	// 	linear_velocity = 0;
	// 	angular_velocity = 0;
	// 	cur_angle = 0;
	// 	tar_angle = 0;
	// 	at_destination = false;
	// 	m_path = std::vector<motion_node>();
	// };

	profile() : linear_velocity(0), max_velocity(50), angular_velocity(0), cur_angle(0), tar_angle(0), distance(1), at_destination(false), m_path(std::vector<motion_node>()){}; //path(std::vector<point>()),

	void follow_path();
	void setCurrent(double new_x, double new_y) { current = point(new_x, new_y); }
	void setCurrentHeading(double new_heading) {cur_angle = new_heading;};
	void setTargetHeading(point target);
	void setTargetAngle(double target);
    void normalizeCurrent(double current);
    void pointTurn();
    void compile_path_linear(std::vector<point> path);
	double getLinearVelocity() {return linear_velocity;};
	double getAngularVelocity() {return angular_velocity;};
	double getMaxVelocity() {return max_velocity;};
	double getAtDestination() {return at_destination;};
	double getStick() {return distance;};

private:
	//Linear and Angular velocities are in percents. The calculation to get those to actual track velocities are in the ROS node
	double linear_velocity; //How fast straight are we going
	double max_velocity;
	double angular_velocity; //How fast left/right are we going
	double cur_angle; //Current heading
	double tar_angle; //Target heading
	double distance; //"Stick" length
	double final_angle; //The angle we want to end up facing
	bool at_destination; //Decides if we are at the destination & want to be point turning

	// std::vector<point> path;
	std::vector<motion_node> m_path;
	point current;
};
