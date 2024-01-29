#pragma once
#include "transfer_types.hpp"

namespace location_system
{

    // Initializes and joins the destination coms system, and links it's lifetime to the lifetime of the calling process.
    void init();

    // Joins the destination coms system
    void join();

    // Gets destination from coms system
    locationF_t get_location();

    // Sets destination on coms system
    void set_location(locationF_t local);

    namespace internals
    {
        void cleanup();

        constexpr const char *SHM_NAME = "LOCATION_SHM";
        constexpr std::size_t SHM_SIZE = sizeof(KingEngine::locationI_t);
    } // namespace internals

}