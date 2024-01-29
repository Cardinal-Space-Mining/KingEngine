#include "destination.hpp"

#include "transfer_types.hpp"
#include "util.hpp"

#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h>    /* For O_* constants */
#include <unistd.h>   // for close

using namespace destination_system;

volatile KingEngine::locationI_t *DST_DATA_PTR = nullptr;

locationF_t destination_system::get_destination()
{
    uint64_t l = *reinterpret_cast<volatile uint64_t *>(DST_DATA_PTR);
    return KingEngine::from_integer_location(*reinterpret_cast<KingEngine::locationI_t *>(&l));
}

void destination_system::set_destination(locationF_t local)
{
    auto tmp = KingEngine::from_float_location(local);
    uint64_t as_int = *reinterpret_cast<uint64_t *>(&tmp);
    reinterpret_cast<volatile uint64_t *>(DST_DATA_PTR)[0] = as_int;
}

void destination_system::join()
{
    // Open FD to shm file
    int shm_fd = shm_open(destination_system::internals::SHM_NAME, O_RDWR, S_IREAD | S_IWRITE);

    if (shm_fd < 0) // shm_open failure
    {
        std::perror("shm_open failed in destination_system::join");
        std::exit(EXIT_FAILURE);
    }

    // Map the file into memory
    void *shared_ptr = mmap(NULL, destination_system::internals::SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    if (shared_ptr == MAP_FAILED)
    {
        std::perror("mmap failed in destination_system::join");
        std::exit(EXIT_FAILURE);
    }

    // Close the shm file descriptor, but the memory remains mapped as long as shm_unlink is not called on the shm area
    if (close(shm_fd) != 0)
    {
        std::perror("close failed in destination_system::join");
        std::exit(EXIT_FAILURE);
    }

    DST_DATA_PTR = reinterpret_cast<KingEngine::locationI_t *>(shared_ptr);
}

void destination_system::internals::cleanup()
{
    if (shm_unlink(destination_system::internals::SHM_NAME) != 0)
    {
        perror("shm_unlink failed in destination_system::internals::cleanup");
    }
}

void destination_system::init()
{
    int shm_fd = shm_open(destination_system::internals::SHM_NAME, O_CREAT | O_RDWR, S_IREAD | S_IWRITE);
    if (shm_fd == -1)
    {
        destination_system::internals::cleanup();
        std::perror("shm_open failed in destination_system::init");
        std::exit(EXIT_FAILURE);
    }

    if (ftruncate(shm_fd, destination_system::internals::SHM_SIZE) != 0)
    {
        std::perror("ftruncate failed in destination_system::init");
        destination_system::internals::cleanup();
        std::exit(EXIT_FAILURE);
    }

    ctrl_c_patch::patch();

    if (atexit(destination_system::internals::cleanup) != 0)
    {
        std::perror("atexit failed in destination_system::init");
        destination_system::internals::cleanup();
        std::exit(EXIT_FAILURE);
    };

    // Map the file into memory
    void *shared_ptr = mmap(NULL, destination_system::internals::SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_ptr == MAP_FAILED)
    {
        std::perror("mmap failed in destination_system::init");
        std::exit(EXIT_FAILURE);
    }

    // Close the shm file descriptor, but the memory remains mapped as long as shm_unlink is not called on the shm area
    if (close(shm_fd) != 0)
    {
        std::perror("close failed in destination_system::init");
        std::exit(EXIT_FAILURE);
    }

    DST_DATA_PTR = reinterpret_cast<KingEngine::locationI_t *>(shared_ptr);

    locationF_t empty = {0};

    destination_system::set_destination(empty);
}