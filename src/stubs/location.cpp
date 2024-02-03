#include "location.hpp"

#include "transfer_types.hpp"
#include "util.hpp"

#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h>    /* For O_* constants */
#include <unistd.h>   // for close
#include <errno.h>

using namespace location_system;

volatile KingEngine::locationI_t *LOCATION_PTR = nullptr;

locationF_t location_system::get_location()
{
    uint64_t l = *reinterpret_cast<volatile uint64_t *>(LOCATION_PTR);
    return KingEngine::from_integer_location(*reinterpret_cast<KingEngine::locationI_t *>(&l));
}

void location_system::set_location(locationF_t local)
{
    auto tmp = KingEngine::from_float_location(local);
    uint64_t as_int = *reinterpret_cast<uint64_t *>(&tmp);
    reinterpret_cast<volatile uint64_t *>(LOCATION_PTR)[0] = as_int;
}

void location_system::join()
{
    // Open FD to shm file
    int shm_fd = shm_open(location_system::internals::SHM_NAME, O_RDWR, S_IREAD | S_IWRITE);

    if (shm_fd < 0) // shm_open failure
    {
        std::perror("shm_open failed in location_system::join");
        std::exit(EXIT_FAILURE);
    }

    // Map the file into memory
    void *shared_ptr = mmap(NULL, location_system::internals::SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    if (shared_ptr == MAP_FAILED)
    {
        std::perror("mmap failed in location_system::join");
        std::exit(EXIT_FAILURE);
    }

    // Close the shm file descriptor, but the memory remains mapped as long as shm_unlink is not called on the shm area
    if (close(shm_fd) != 0)
    {
        std::perror("close failed in location_system::join");
        std::exit(EXIT_FAILURE);
    }

    LOCATION_PTR = reinterpret_cast<KingEngine::locationI_t *>(shared_ptr);
}

void location_system::internals::cleanup()
{
    if (shm_unlink(location_system::internals::SHM_NAME) != 0)
    {
        perror("shm_unlink failed in location_system::internals::cleanup");
    }
}

void location_system::init()
{
    int shm_fd = shm_open(location_system::internals::SHM_NAME, O_CREAT | O_RDWR, S_IREAD | S_IWRITE);
    if (shm_fd == -1)
    {
        location_system::internals::cleanup();
        std::perror("shm_open failed in location_system::init");
        std::exit(EXIT_FAILURE);
    }

ftruncate_again:
    if (ftruncate(shm_fd, location_system::internals::SHM_SIZE) != 0)
    {
        if (errno == EINTR)
        {
            goto ftruncate_again;
        }

        std::perror("ftruncate failed in destination_system::init");
        location_system::internals::cleanup();
        std::exit(EXIT_FAILURE);
    }

    ctrl_c_patch::patch();

    if (atexit(location_system::internals::cleanup) != 0)
    {
        std::perror("atexit failed in location_system::init");
        location_system::internals::cleanup();
        std::exit(EXIT_FAILURE);
    };

    // Map the file into memory
    void *shared_ptr = mmap(NULL, location_system::internals::SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_ptr == MAP_FAILED)
    {
        std::perror("mmap failed in location_system::init");
        std::exit(EXIT_FAILURE);
    }

    // Close the shm file descriptor, but the memory remains mapped as long as shm_unlink is not called on the shm area
    if (close(shm_fd) != 0)
    {
        std::perror("close failed in location_system::init");
        std::exit(EXIT_FAILURE);
    }

    LOCATION_PTR = reinterpret_cast<KingEngine::locationI_t *>(shared_ptr);

    locationF_t empty = {};

    location_system::set_location(empty);
}