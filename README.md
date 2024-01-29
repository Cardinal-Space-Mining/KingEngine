# KingEngine
Makes strategic decisions and makes the robot do things.
It contains stubs for each subsystem to communicate.

# Current Architecture
![Controls Architecture Diagram](https://github.com/Cardinal-Space-Mining/KingEngine/assets/84042085/90bf5606-f1f2-4c98-8338-3e8f9f94cc49)

# Units and Stub Communication Implementation
We are using meters as units for location data. That suggests we use floats. However, there are no fixed-width float types. We want to be able to fix one location point inside a machine word (64 bits) for inter-process transfer. Thus, I convert the floating meter number into an integer number of millimeters and transfer that point.

# Namespaces and Stubs
Stubs are the pieces that can be incorporated into other projects for communication. In the stubs file, everything in headerfiles and the global namespace is for use by anyone. Anything in the headers and the KingEngine namespace is for internal use only. Beware of dragons if touching anything in the cpp implementation files.

# Shared Memory Python Implementation
From what I can find, [multiprocessing.shared_memory](https://docs.python.org/3/library/multiprocessing.shared_memory.html) implemented [here](https://github.com/python/cpython/blob/3.12/Lib/multiprocessing/shared_memory.py) is the best way to access shared memory spaces. It relies on an internal module called _posixshmem which is defined [here](https://github.com/python/cpython/blob/3.12/Modules/_multiprocessing/posixshmem.c). Looking at it, it converts the Python string to a Unicode UTF8 using PyUnicode_AsUTF8, and UTF8 should be compatible with ASCII. It then calls [shm_open](https://man7.org/linux/man-pages/man3/shm_open.3.html) with that Unicode string to get a file descriptor to the backing file, but fails with a "named file not found" type error.


