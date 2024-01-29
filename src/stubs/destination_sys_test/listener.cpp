#include <iostream>

#include "destination.hpp" //destination_system
#include "transfer_types.hpp" //locationF_t

int main()
{
    destination_system::init();

    while (true)
    {
        locationF_t l;
        std::cout << "Give x: ";
        std::cin >> l.x_m;
        std::cout << "Give y: ";
        std::cin >> l.y_m;
        destination_system::set_destination(l);
    }

    return 0;
}