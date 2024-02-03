#pragma once
#include <functional>
#include "transfer_types.hpp"
#include <atomic>
#include <unistd.h>
#include <limits>

#include <semaphore.h>

namespace recalc_notif_sys
{
    // Initializes the notification system, joins the system but does not suscribe, and links it's lifetime to the lifetime of the calling process.
    void init();

    // Joins the notification system which allows the process to send notifications
    void join();

    // Notifies the network
    void notify();

    // Suscribes to notifications and sets a handler to be run everytime the system is notified. Calling this requires initializing the destination system.
    void set_event_handler(std::function<void(locationF_t)> func);

    // Internals.
    namespace internals
    {
        typedef struct notif_backer
        {
            sem_t notif_sem;
            uint16_t num_processes;
            pid_t processes[std::numeric_limits<short>::max()];
        } notif_backer;

        constexpr auto SHM_NAME = "RECALCULATE_NAME";
        constexpr auto SHM_SIZE = sizeof(notif_backer);

        void notif_sys_cleanup();

        void event_hdlr_shim(int i);

        void join_process_list();

        void unsubscribe(pid_t pid);

        void unsubscribe_self();

        void subscribe();

        void raise_signal(int signal);

    } // namespace internals
} // namespace recalc_notif_sys
