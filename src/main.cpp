#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

int main() {
    // Create a shared memory object (if not exists)
    int shm_fd = shm_open("/example", O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        return 1;
    }

    // Set the size of the shared memory object (1 byte in this example)
    if (ftruncate(shm_fd, 1) == -1) {
        perror("ftruncate");
        return 1;
    }

    // Map the shared memory object into the process's address space
    void *ptr = mmap(0, 1, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    // Get information about the shared memory object
    struct stat sb;
    if (fstat(shm_fd, &sb) == -1) {
        perror("fstat");
        return 1;
    }

    // Display the maximum size of the shared memory object
    printf("Maximum size of shared memory segment: %lld bytes\n", (long long)sb.st_size);

    // Unmap the shared memory object
    if (munmap(ptr, 1) == -1) {
        perror("munmap");
        return 1;
    }

    // Close and unlink the shared memory object (optional)
    close(shm_fd);
    shm_unlink("/example");

    return 0;
}
