#include <cstdio> //printf

#include <unistd.h> //sleep

#include "destination.hpp"    //destination_system
#include "notif_sys.hpp"      //notif_sys
#include "transfer_types.hpp" //locationF_t

void notif_hdlr(locationF_t data)
{
    std::printf("(%f, %f)\n", data.x_m, data.y_m);
}

int main()
{
    destination_system::join();
    recalc_notif_sys::join();
    recalc_notif_sys::set_event_handler(notif_hdlr);

    while (true)
    {
        sleep(1);
    }
}