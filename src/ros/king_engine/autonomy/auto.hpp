#pragma once
#include <rclcpp/rclcpp.hpp>
typedef struct Location
{
	double x;
	double y;
} Location;

class AutonomyNode : public rclcpp::Node 
{
public:
	AutonomyNode();
	~AutonomyNode();

protected:
	bool is_autonomous(){return is_autonomy_enabled;}
	bool set_autonomy(bool status){is_autonomy_enabled = status;}

	Location get_location()
	{
		current_location = //TODO: get localization data
	}

	void set_target_location(Location loc)
	{
		target_location = loc;
	}

	Location get_target_location()
	{
		return target_location;
	}


private:
	bool is_autonomy_enabled;
	Location current_location;
	Location target_location;
	
}
