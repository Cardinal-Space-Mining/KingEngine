#include <iostream> //std::cout

#include "destination.hpp" //destination_system
#include "notif_sys.hpp"   //notif_sys

#include <signal.h>

int main()
{
    destination_system::init();
    recalc_notif_sys::init();

    //return 0;

    while (true)
    {
        locationF_t l;
        std::cout << "Give x: ";
        std::cin >> l.x_m;
        std::cout << "Give y: ";
        std::cin >> l.y_m;
        destination_system::set_destination(l);
        recalc_notif_sys::notify();
        if (l.x_m == 0.0 && l.y_m == 0.0)
        {
            recalc_notif_sys::internals::raise_signal(SIGKILL);
            return 0;
        }
        
    }

    return 0;
}