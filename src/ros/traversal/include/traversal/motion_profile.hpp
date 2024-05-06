#pragma once
// STL includes
//#include "path_plan/WeightMap.hpp"
#include <ostream>	  //std::ostream
#include <stdint.h>	  //uint16_t, uint_fast16_t
#include <string>	  //std::string
#include <vector>	  //std::vector
#include <utility>	  //std::pair
#include <queue>	  //std::queue
#include <functional> //std::reference_wrapper
#include <vector>
#include <stdint.h>	  //uint16_t, uint_fast16_t


using path_t = std::vector<point>;

using motion_path = std::vector<motion_node>;

//using point = WeightMap::point_t;
using point = std::pair<uint16_t, uint16_t>;



class motion_node {

	motion_node(const point& a, const point& b) : a(a), b(b) {}

	motion_node() {}

	~motion_node() {}

public:

	virtual double get_tangent_angle(point p) = 0;

	virtual point get_closest_point(point current) = 0;

private:

	point a;

	point b;

public:

	bool operator==(const motion_node& other) const
	{
		return a == other.a && b == other.b;
	}
	
};


class linear_motion : class motion_node {

	linear_motion(point a, point b) : motion_node(a, b);


	linear_motion() = default;
};

class hypocycloid_motion : public motion_node {

	double radius;

public:

	hypocycloid_motion(const point& a, const point& b, double radius) 
		: motion_node(), a(a), b(b), radius(radius) {}

};

class bezier_motion : public motion_node {
	point p0;
	point p1;
	point p2;
	point p3;

public:

	bezier_motion(const point& p0, const point& p1,
		const point& p2, const point& p3)
		: p0(p0), p1(p1), p2(p2), p3(p3) {}


	bezier_motion() = default;
};


class profile {

public:

	profile(path_t& path) : path(path) {
		linear_velocity = 0;
		angular_velocity = 0;
		cur_angle = 0;
		tar_angle = 0;
	};

	double linear_velocity;

	double angular_velocity;

	double cur_angle;

	double tar_angle;

	path_t& path;

	motion_path motion_path;

	point current; 



	std::pair<double, double> get_setspeed();

	void compile_path_linear(std::vector<point_t> path);

	void compile_path_hypocycloid(std::vector<point_t> path);

	void compile_path_bezier(std::vector<point_t> path);


	void follow_path();

		
private:

	int v_max;

	int v_min;

};