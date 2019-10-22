## Introduction

For this assignment, we leave the realm of deformable objects altogether, instead focusing on objects that do not deform at all. Such rigid body models gain some simplicity via the avoidance of potential energy functions but this is somewhat offset by more complicated generalized coordinates.  

Specifically, we will implement an unconstrained rigid body (an exciting torus!) that you can click and pull around. 

### Prerequisite installation

On all platforms, we will assume you have installed cmake and a modern c++
compiler on Mac OS X[¹](#¹macusers), Linux[²](#²linuxusers), or
Windows[³](#³windowsusers).

We also assume that you have cloned this repository using the `--recursive`
flag (if not then issue `git submodule update --init --recursive`). 

**Note:** We only officially support these assignments on Ubuntu Linux 18.04 (the OS the teaching labs are running) and OSX 10.13 (the OS I use on my personal laptop). While they *should* work on other operating systems, we make no guarantees. 

**All grading of assignments is done on Linux 18.04**

### Layout

All assignments will have a similar directory and file layout:

    README.md
    CMakeLists.txt
    main.cpp
    assignment_setup.h
    include/
      function1.h
      function2.h
      ...
    src/
      function1.cpp
      function2.cpp
      ...
    data/
      ...
    ...

The `README.md` file will describe the background, contents and tasks of the
assignment.

The `CMakeLists.txt` file setups up the cmake build routine for this
assignment.

The `main.cpp` file will include the headers in the `include/` directory and
link to the functions compiled in the `src/` directory. This file contains the
`main` function that is executed when the program is run from the command line.

The `include/` directory contains one file for each function that you will
implement as part of the assignment.

The `src/` directory contains _empty implementations_ of the functions
specified in the `include/` directory. This is where you will implement the
parts of the assignment.

The `data/` directory contains _sample_ input data for your program. Keep in
mind you should create your own test data to verify your program as you write
it. It is not necessarily sufficient that your program _only_ works on the given
sample data.

## Compilation for Debugging

This and all following assignments will follow a typical cmake/make build
routine. Starting in this directory, issue:

    mkdir build
    cd build
    cmake ..

If you are using Mac or Linux, then issue:

    make

## Compilation for Testing

Compiling the code in the above manner will yield working, but very slow executables. To run the code at full speed, you should compile it in release mode. Starting in the **build directory**, do the following:

    cmake .. -DCMAKE_BUILD_TYPE=Release
    
Followed by:

    make 
  
Your code should now run significantly (sometimes as much as ten times) faster. 

If you are using Windows, then running `cmake ..` should have created a Visual Studio solution file
called `a5-rigid-bodies.sln` that you can open and build from there. Building the project will generate an .exe file.

Why don't you try this right now?

## Execution

Once built, you can execute the assignment from inside the `build/` using 

    ./a5-rigid-bodies

While running, you can reset the position of the rigid body by pressing `r`. 

## Background 
test

## Assignment Implementation

### Implementation Notes

### rodrigues.cpp

The rodrigues formula for computing the marix exponential of a $3\times 3$, skew-symmetric matrix.

### rigid_to_world.cpp

The rigid transformation from the undeformed space to the world (deformed) space.

### rigid_body_jacobian.cpp

The Jacobian of the rigid-to-world transform. 

### inertia_matrix.cpp

Compute the rigid inertia matrix of a 3d object, represented by a surface mesh, via surface only integration.

### pick_nearest_vertices.cpp

**Use your code from the previous assignments**

### dV_spring_particle_particle_dq.cpp

**Use your code from the previous assignments**

### exponential_euler.h

Implement the explicit, exponential Euler time integration scheme. 
