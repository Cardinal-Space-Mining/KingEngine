#include "location.hpp" //location_system

#include <cstdio> //printf

#include <unistd.h> //sleep

int main()
{
    location_system::init();

    while (true)
    {
        auto data = location_system::get_location();
        std::printf("(%f, %f)\n", data.x_m, data.y_m);
        sleep(1);
    }
}