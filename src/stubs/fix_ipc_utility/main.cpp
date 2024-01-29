#include "destination.hpp"
#include "notif_sys.hpp"

int main(){
    //Systems are automatically closed in accordance with atexit. Yay
    destination_system::init();
    recalc_notif_sys::init();
    return 0;
}