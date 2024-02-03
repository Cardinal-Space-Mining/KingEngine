#include "notif_sys.hpp"
#include "destination.hpp"
#include "util.hpp"

#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <cstring>

#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h>    /* For O_* constants */
#include <unistd.h>   // for close
#include <signal.h>   //for kill
#include <errno.h>

recalc_notif_sys::internals::notif_backer *backing_struct = nullptr;

void recalc_notif_sys::internals::notif_sys_cleanup()
{
    if (sem_destroy(&backing_struct->notif_sem) != 0)
    {
        perror("sem_destroy errored in recalc_notif_sys::internals::notif_sys_cleanup");
    }

    if (shm_unlink(recalc_notif_sys::internals::SHM_NAME) != 0)
    {
        perror("shm_unlink errored in recalc_notif_sys::internals::notif_sys_cleanup");
    }
}

void recalc_notif_sys::init()
{
    using recalc_notif_sys::internals::notif_backer;
    using recalc_notif_sys::internals::notif_sys_cleanup;
    using recalc_notif_sys::internals::SHM_NAME;
    using recalc_notif_sys::internals::SHM_SIZE;

    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, S_IREAD | S_IWRITE);
    if (shm_fd == -1)
    {
        notif_sys_cleanup();
        std::perror("shm_open in recalc_notif_sys::init");
        std::exit(EXIT_FAILURE);
    }

ftruncate_again:
    if (ftruncate(shm_fd, SHM_SIZE) != 0)
    {
        if (errno == EINTR)
        {
            goto ftruncate_again;
        }

        std::perror("ftruncate failed in recalc_notif_sys::init");
        destination_system::internals::cleanup();
        std::exit(EXIT_FAILURE);
    }

    ctrl_c_patch::patch();

    if (atexit(notif_sys_cleanup) != 0)
    {
        notif_sys_cleanup();
        std::perror("atexit in recalc_notif_sys::init");
        std::exit(EXIT_FAILURE);
    };

    void *shared_ptr = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_ptr == MAP_FAILED)
    {
        std::perror("mmap in recalc_notif_sys::init");
        std::exit(EXIT_FAILURE);
    }

    // Use placement new to construct a struct instance inside shared memory
    backing_struct = new (shared_ptr) notif_backer;

    // Initialize the semaphore to be open and shared between processes
    if (sem_init(&backing_struct->notif_sem, 1, 1) != 0)
    {
        std::perror("sem_init in recalc_notif_sys::init");
        std::exit(EXIT_FAILURE);
    }

    if (close(shm_fd) != 0)
    {
        std::perror("close in recalc_notif_sys::init");
        std::exit(EXIT_FAILURE);
    };
}

void recalc_notif_sys::join()
{
    int shm_fd = shm_open(recalc_notif_sys::internals::SHM_NAME, O_RDWR, S_IREAD | S_IWRITE);
    if (shm_fd == -1)
    {
        std::perror("shm_open in recalc_notif_sys::join");
        std::exit(EXIT_FAILURE);
    }

    void *shared_ptr = mmap(NULL, recalc_notif_sys::internals::SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_ptr == MAP_FAILED)
    {
        std::perror("mmap in recalc_notif_sys::join");
        std::exit(EXIT_FAILURE);
    }

    backing_struct = reinterpret_cast<recalc_notif_sys::internals::notif_backer *>(shared_ptr);

    if (close(shm_fd) != 0)
    {
        std::perror("close in recalc_notif_sys::join");
        std::exit(EXIT_FAILURE);
    };
}

void recalc_notif_sys::notify()
{
    recalc_notif_sys::internals::raise_signal(SIGUSR1);
}

std::function<void(locationF_t)> fn = [](locationF_t)
{
    std::printf("Warning, default function called");
    std::exit(EXIT_FAILURE);
};

void recalc_notif_sys::internals::event_hdlr_shim(int i)
{
    if (i == SIGUSR1)
    {
        auto dst = destination_system::get_destination();
        fn(dst);
    };
}

void recalc_notif_sys::internals::join_process_list()
{
sem_wait_again:
    if (sem_wait(&backing_struct->notif_sem) != 0)
    {
        if (errno == EAGAIN || errno == EINTR)
        {
            goto sem_wait_again;
        }

        std::perror("sem_wait in recalc_notif_sys::internals::join_process_list");
        std::exit(EXIT_FAILURE);
    }

    pid_t pid = getpid();

    uint16_t idx = backing_struct->num_processes;
    backing_struct->num_processes = backing_struct->num_processes + 1;
    backing_struct->processes[idx] = pid;

    if (sem_post(&backing_struct->notif_sem) != 0)
    {
        std::perror("sem_post in recalc_notif_sys::internals::join_process_list");
        std::exit(EXIT_FAILURE);
    }
}

void recalc_notif_sys::internals::unsubscribe(pid_t pid)
{
sem_wait_again:
    if (sem_wait(&backing_struct->notif_sem) != 0)
    {
        if (errno == EAGAIN || errno == EINTR)
        {
            goto sem_wait_again;
        }

        std::perror("sem_wait in recalc_notif_sys::internals::unsubscribe");
        std::exit(EXIT_FAILURE);
    }
    for (size_t i = 0; i < backing_struct->num_processes; i++)
    {
        if (backing_struct->processes[i] == pid)
        {
            backing_struct->processes[i] = backing_struct->processes[backing_struct->num_processes - 1];
            backing_struct->processes[backing_struct->num_processes - 1] = 0;
        }
    }

    if (sem_post(&backing_struct->notif_sem) != 0)
    {
        std::perror("sem_post in recalc_notif_sys::internals::unsubscribe");
        std::exit(EXIT_FAILURE);
    }
}

void recalc_notif_sys::internals::unsubscribe_self()
{
    pid_t pid = getpid();
    unsubscribe(pid);
}

void recalc_notif_sys::internals::subscribe()
{
    struct sigaction newact = {};

    newact.sa_handler = event_hdlr_shim;

    if (sigaction(SIGUSR1, &newact, NULL) != 0)
    {
        std::perror("sigaction in recalc_notif_sys.cpp::subscribe");
        std::exit(EXIT_FAILURE);
    }

    ctrl_c_patch::patch();

    join_process_list();

    atexit(unsubscribe_self);
}

bool subscribed = false;

void recalc_notif_sys::set_event_handler(std::function<void(locationF_t)> func)
{
    fn = func;

    if (!subscribed)
    {
        recalc_notif_sys::internals::subscribe();
        subscribed = true;
    }
}

void recalc_notif_sys::internals::raise_signal(int signal)
{
    if (sem_wait(&backing_struct->notif_sem) != 0)
    {
        std::perror("sem_wait in recalc_notif_sys::notify");
        std::exit(EXIT_FAILURE);
    }

    for (size_t i = 0; i < backing_struct->num_processes; i++)
    {
#ifdef DEBUG_PRINT
        std::printf("Issueing signal %d to pid %d\n", SIGUSR1, backing_struct->processes[i]);
#endif
    start:
        if (kill(backing_struct->processes[i], signal) != 0)
        {
            if (errno == ESRCH)
            {
                printf("Warning, process with pid %d does not seem to exist. They have been automatically unsubscribed.", backing_struct->processes[i]);
                recalc_notif_sys::internals::unsubscribe(backing_struct->processes[i]);
                goto start;
            }

            char str[1024];
            // TODO maybe look up process name at this point and add it to error messaging
            std::snprintf(str, 1024, "%s, %ld", "kill in recalc_notif_sys::notify on PID ", static_cast<long int>(backing_struct->processes[i]));
            std::perror(str);
            std::exit(EXIT_FAILURE);
        }
    }

    if (sem_post(&backing_struct->notif_sem) != 0)
    {
        auto errmsg = "sem_post in recalc_notif_sys::internals::raise_signal";
        std::perror(errmsg);
        std::exit(EXIT_FAILURE);
    }
}
