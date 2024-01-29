#include "destination.hpp" //destination_system

#include <cstdio> //printf

#include <unistd.h> //sleep

int main()
{
    destination_system::join();

    while (true)
    {
        auto data = destination_system::get_destination();
        std::printf("(%f, %f)\n", data.x_m, data.y_m);
        sleep(1);
    }
}