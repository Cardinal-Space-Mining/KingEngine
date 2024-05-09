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

struct point {
	double x, y;

	bool operator==(const point& b) const {
		return ((x > b.x - 0.01 && x < b.x + 0.01) && (y > b.y - 0.01 && y < b.y + 0.01));
	}

	point(double x, double y) : x(x), y(y) {};
};

class motion_node {
public:
	virtual double get_tangent_angle(point p) = 0;
	virtual point get_closest_point(point current) = 0;
	virtual point get_target_from_distance(point current, double distance) = 0;

	point get_start() { return a; }
	point get_end() { return b; }

	bool operator==(const motion_node& other) const {
		return a == other.a && b == other.b;
	}

protected:
	motion_node(const point& a, const point& b) : a(a), b(b) {}
	motion_node() {}
	~motion_node() {}

private:
	point a;
	point b;
};

class linear_motion : public motion_node {
public:
	linear_motion(point a, point b) : motion_node(a, b) {}
	linear_motion() = default;
};

class profile {
public:
	profile(std::vector<point>* path) : path(path) {
		linear_velocity = 0;
		angular_velocity = 0;
		cur_angle = 0;
		tar_angle = 0;
		at_destination = false;
		m_path = nullptr;
	};

	profile() : linear_velocity(0), angular_velocity(0), cur_angle(0), tar_angle(0), distance(0), at_destination(false), path(nullptr), m_path(nullptr) {};

	void follow_path();
	void setCurrent(double new_x, double new_y) { current = point(new_x, new_y); }
	void setHeading(point target);
	void setTargetAngle(double target);
	void pointTurn();
	void compile_path_linear(std::vector<point> path);

	std::pair<double, double> get_speed();

private:
	double linear_velocity;
	double angular_velocity;
	double cur_angle;
	double tar_angle;
	double distance;
	double final_angle;
	bool at_destination;

	std::vector<point>* path;
	std::vector<motion_node>* m_path;
	point current; 
};
