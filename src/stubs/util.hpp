
#include <signal.h>
#include <cstdlib>
#include <cstdio>

// This changes the signal handler for ctrl-c to terminate the process normally instead of abnormally. This results in atexit functions being called and cleanup taking place.
namespace ctrl_c_patch
{
    inline void patch_hdlr(int signal)
    {
        if (signal == SIGINT)
        {
            std::exit(EXIT_FAILURE);
            _exit(EXIT_FAILURE); // std::exit is not gurenteed to work in a signal handler. Sadness
        }
    }

    inline void patch()
    {
        struct sigaction newact = {0};

        newact.sa_handler = patch_hdlr;

        if (sigaction(SIGINT, &newact, NULL) != 0)
        {
            std::perror("sigaction in ctrl_c_patch::patch");
            std::exit(EXIT_FAILURE);
        }
    }
} // namespace ctrl_c_patch
