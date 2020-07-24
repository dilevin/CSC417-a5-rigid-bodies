[![](https://github.com/dilevin/CSC2549-a5-rigid-bodies/workflows/Build-CSC2549-Assignment-Five/badge.svg)](https://github.com/dilevin/CSC2549-a5-rigid-bodies/actions)

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

In this assignment we will implement a physics simulation of an unconstrained rigid body in low gravity (e.g. space [donut](https://www.youtube.com/watch?v=8-4P1WPE-Qg) which you can interactively fling around the world. The goal is to get a good handle on the kinematics and dynamics of rigid body mechanics, which we will extend in the final assignment to handle collision resolution. Rigid bodies are the first type of object we will encounter that use a truly generalized, generalized coordinate (i.e not just the vertex positions of the mesh) and this complicates both their mathematical treatment and implementation. Let's dive right in!

![Fun with interactive rigid bodies](images/rb_example.gif)

## Resources

Like cloth simulation, comprehensive, readable resources about rigid body dynamics and simulation are hard to find. Some people reccomend this [book](https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-complete.pdf), which is quite mathematical. This [paper](https://animation.rwth-aachen.de/media/papers/2012-EG-STAR_Rigid_Body_Dynamics.pdf) provides a detailed overview of rigid body simulation with contact, but is also very equation heavy. In these notes I'll try to provide a one-stop basic introduction to rigid body dynamics, leaning heavily (as usual) on the variational approach. The rotation matrix derivative is more or less a reproduction of the wonderful paper [here](https://www.cis.upenn.edu/~cjtaylor/PUBLICATIONS/pdfs/TaylorTR94b.pdf).

## Generalized Coordinates and Velocities 

### Generalized Coordinates for Rigid Motion

As with all previous assignments, we begin our journey into rigid body simulation by searching for approriate generalized coordinates. Once we have these, all remaining kinematic and dynamic relationships can be derived by turning our variational crank. Rigid bodies are going to be the first time we see generalized coordinates that are not just vertex positions. Why is that ?

The rigid body model is an *approximation* which we use for objects that do not deform meaningful in the simulated scenario. To be clear **everything deforms** and everything is deforming all the time, its just often so small that we can't see it, nor does it effect the salient behaviour of our object of interest. In these cases it would be a big waste (of time and memory) to use something like [finite elements](https://github.com/dilevin/CSC2549-a3-finite-elements-3d) to simulate said object. So instead we choose generalized coordinates that do not allow the object to deform at all. 

Previously when we discussed deformation, we considered how the squared length of a vector in some undeformed space changes under a deformation mapping. What we learned is that, given this undeformed vector <img src="/tex/fe4e62694c7a379a0b31c6aa14046dd0.svg?invert_in_darkmode&sanitize=true" align=middle width=24.79439324999999pt height=22.831056599999986pt/>, its deformed squared length became 

<p align="center"><img src="/tex/b2b7cbacb79d0381d54c658b27ea49f2.svg?invert_in_darkmode&sanitize=true" align=middle width=154.91169209999998pt height=19.789994399999998pt/></p>

where <img src="/tex/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode&sanitize=true" align=middle width=12.85392569999999pt height=22.465723500000017pt/> is the deformation gradient of the deformation mapping. A good question to ask is *what is required of F so that <img src="/tex/22af0a0b1f1e312766dc254f318fd3a6.svg?invert_in_darkmode&sanitize=true" align=middle width=99.38326364999999pt height=31.360807499999982pt/> ? Well, we want <img src="/tex/5eee945a9f27902f8ae0bf619943be32.svg?invert_in_darkmode&sanitize=true" align=middle width=66.49705589999998pt height=27.6567522pt/>, where <img src="/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> is the identity matrix (if you've taken [CSC418](https://github.com/dilevin/computer-graphics-csc418) then you've seen this argument before but wrt the transformation of normal vectors). This property, that the transpose of a matrix is also its inverse, is associated with the special class of matrices called [**orthogonal matrices**](https://en.wikipedia.org/wiki/Orthogonal_matrix). Orthogonal matrices thus represent transformations that preserve the distance between all points being transformed. In other words, they do not allow deformation (just what we were looking for). 

Additionally, we've all learned the painful lesson that we don't want our simulated objects to turn inside out. So we'd also like the determinant of <img src="/tex/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode&sanitize=true" align=middle width=12.85392569999999pt height=22.465723500000017pt/> to be positive. This eliminates such transformations as reflections, leaving us with rotations as the only valid form of <img src="/tex/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode&sanitize=true" align=middle width=12.85392569999999pt height=22.465723500000017pt/>. Because <img src="/tex/a6d6e9a27408b5dc28e388ff6b4d1963.svg?invert_in_darkmode&sanitize=true" align=middle width=92.1075573pt height=28.92634470000001pt/> (here <img src="/tex/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode&sanitize=true" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is a rotation matrix), then

<p align="center"><img src="/tex/d5204ea6053d2178410e7f0656a802ec.svg?invert_in_darkmode&sanitize=true" align=middle width=95.176488pt height=17.0612508pt/></p>

where <img src="/tex/980fcd4213d7b5d2ffcc82ec78c27ead.svg?invert_in_darkmode&sanitize=true" align=middle width=10.502226899999991pt height=14.611878600000017pt/> is a constant of integration and is a rigid body translation. This shows us that the world space position, <img src="/tex/86b983159e30766ec90abefd5a30c50d.svg?invert_in_darkmode&sanitize=true" align=middle width=14.942908199999989pt height=26.085962100000025pt/> of any point in the undeformed space <img src="/tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode&sanitize=true" align=middle width=14.29216634999999pt height=22.55708729999998pt/> is given by the familiar rigid body transformation. This transformation is parameterized by <img src="/tex/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode&sanitize=true" align=middle width=12.60847334999999pt height=22.465723500000017pt/> and <img src="/tex/980fcd4213d7b5d2ffcc82ec78c27ead.svg?invert_in_darkmode&sanitize=true" align=middle width=10.502226899999991pt height=14.611878600000017pt/> and so we will choose these as our generalized coordinates <img src="/tex/e73485aa867794d51ccd8725055d03a3.svg?invert_in_darkmode&sanitize=true" align=middle width=9.97711604999999pt height=14.611878600000017pt/>. Now rather than <img src="/tex/e73485aa867794d51ccd8725055d03a3.svg?invert_in_darkmode&sanitize=true" align=middle width=9.97711604999999pt height=14.611878600000017pt/> being a set of vertex position, it is a set of rotations and translations for each rigid body in our physical system. 

### Generalized Velocities for Rigid Motion

Now that we know how to find the world space position of any point, <img src="/tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode&sanitize=true" align=middle width=14.29216634999999pt height=22.55708729999998pt/> in a rigid body, we can take the time derivative to get the generalized velocities.

<p align="center"><img src="/tex/deb45b46fcff39421fb34dea50387c3e.svg?invert_in_darkmode&sanitize=true" align=middle width=256.4394129pt height=64.62670829999999pt/></p>

Derivatives of rotation matrices are scary, at least I find them scary. That scariness comes from the inherent constraint that <img src="/tex/e8a9fac2daa78f3ba56e737538bd8fe1.svg?invert_in_darkmode&sanitize=true" align=middle width=53.82579674999999pt height=27.6567522pt/>. Any derivative we take as to respect this. One way to do this is to exploit the fact that rotation matrices are part of a special group of matrices called the [*Special Orthogonal Group*](https://en.wikipedia.org/wiki/Orthogonal_group) and are also a [Lie Group](https://en.wikipedia.org/wiki/Lie_group). Lie Groups can be parameterized by an exponential map that takes an element from the Lie Algebra (a linear space) onto the manifold that represents the Lie Group. A convenient fact about rotation matrices is that the can be represented as 

<p align="center"><img src="/tex/39279d4bd6b5fd1cb6cc8b02d699d6fa.svg?invert_in_darkmode&sanitize=true" align=middle width=105.78189764999999pt height=16.438356pt/></p>

Here <img src="/tex/edf2d17223368c336d02825a95fd04de.svg?invert_in_darkmode&sanitize=true" align=middle width=38.81292029999999pt height=14.15524440000002pt/> is the [matrix exponential](https://en.wikipedia.org/wiki/Matrix_exponential), <img src="/tex/d303788ea8b3ff5079316016e37bf19e.svg?invert_in_darkmode&sanitize=true" align=middle width=7.785368249999991pt height=14.611878600000017pt/> is a <img src="/tex/6018bf12266e0674a19d0a622989ad88.svg?invert_in_darkmode&sanitize=true" align=middle width=36.52961069999999pt height=21.18721440000001pt/> vector and <img src="/tex/33505fe364e795090c1cae20ed8dbb16.svg?invert_in_darkmode&sanitize=true" align=middle width=16.917816299999988pt height=24.65753399999998pt/> is the <img src="/tex/9f2b6b0a7f3d99fd3f396a1515926eb3.svg?invert_in_darkmode&sanitize=true" align=middle width=36.52961069999999pt height=21.18721440000001pt/>, [skew-symmetric](https://en.wikipedia.org/wiki/Skew-symmetric_matrix) matrix created from that vector.We can exploit this formulation to take derivatives in a (slightly) less onerous fashion. As a note, this sort of derivative is often referred to as "geometrically-aware" since it will respect the geometry of the Lie Group. 

Ok, lets proceed by infinitesimally perturbing our rotation matrix. We can do this by adding an infinitesimal multiple of another skew-symmetric matrix <img src="/tex/c98bcf01d6c1e34e5acef27209258898.svg?invert_in_darkmode&sanitize=true" align=middle width=30.61648919999999pt height=24.65753399999998pt/>, which gives us

<p align="center"><img src="/tex/30e2fe6e787f57bac074fc17100134d4.svg?invert_in_darkmode&sanitize=true" align=middle width=216.2040672pt height=16.438356pt/></p>

where the scalar, <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> controls the magnitude of the perturbation. Since this perturbation is meant to be infinitesimally small, <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> is limited towards <img src="/tex/29632a9bf827ce0200454dd32fc3be82.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=21.18721440000001pt/>. 

What remains is to deal with <img src="/tex/edf2d17223368c336d02825a95fd04de.svg?invert_in_darkmode&sanitize=true" align=middle width=38.81292029999999pt height=14.15524440000002pt/>. To do this we are going to replace the exponential operator with its infinite series. This let's us concoct the following:

<p align="center"><img src="/tex/0ea0c13551e85407a80f292c996cdc9d.svg?invert_in_darkmode&sanitize=true" align=middle width=511.3696731pt height=39.452455349999994pt/></p>

Now we are going to compute <img src="/tex/9f7f049e9869245ec7fd94ce4d295b9d.svg?invert_in_darkmode&sanitize=true" align=middle width=145.80577439999996pt height=28.92634470000001pt/>. This is called a [differential](https://en.wikipedia.org/wiki/Differential_of_a_function) and it represents a directional derivative, the change in a function if you move in a particular direction. In this case our direction is <img src="/tex/cf6790d96b3a957f95886f2277062e8b.svg?invert_in_darkmode&sanitize=true" align=middle width=30.61648919999999pt height=24.65753399999998pt/>. The differential is really doing two things at once. The first is computing the directional derivative around some point <img src="/tex/32992401dfceee4234efda2beda8ad42.svg?invert_in_darkmode&sanitize=true" align=middle width=80.94165419999999pt height=24.65753399999998pt/>, while the second is using the limit to "move" the point at which the derivative is evaluated back to <img src="/tex/33505fe364e795090c1cae20ed8dbb16.svg?invert_in_darkmode&sanitize=true" align=middle width=16.917816299999988pt height=24.65753399999998pt/>. **NOTE:** this limit is incredibly helpful. It means we can ignore any terms in the above equation that are functions of <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, since it will become zero. 

A little bit of exciting calculus, followed by some less exciting algebra should convince you that 

<p align="center"><img src="/tex/a1722ef6fbaffde8a9c741c8fc12b239.svg?invert_in_darkmode&sanitize=true" align=middle width=441.9002478pt height=39.452455349999994pt/></p>

But wait, there's more !! We can simplify a bit further by realizing that <img src="/tex/8fab8e4b81e8c8d7934a51f9f70fd7bc.svg?invert_in_darkmode&sanitize=true" align=middle width=8.21920935pt height=14.15524440000002pt/> has survived the differential in its infinite series form (maybe not surprising given how exponentials usually  persist through derivatives). So at the end of it all we get

<p align="center"><img src="/tex/4ab885c60ba4e2bb3c0f38cde36d780a.svg?invert_in_darkmode&sanitize=true" align=middle width=265.90464284999996pt height=33.81208709999999pt/></p>

Which is not half bad. For the full details of these sorts of derivations, I cannot reccomend this [paper](https://www.cis.upenn.edu/~cjtaylor/PUBLICATIONS/pdfs/TaylorTR94b.pdf) enough.

Now our remaining task is to figure out how this relates to the time derivative we were **originally trying to take**. 

#### What does it all mean ? 

Let's start reinterpreting all of this through a physical lens, starting with <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>. In our case, we aren't differentiating wrt to an arbitrary parameter, but rather by a time. This means, for us, <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> is really <img src="/tex/4f4f4e395762a3af4575de74c019ebb5.svg?invert_in_darkmode&sanitize=true" align=middle width=5.936097749999991pt height=20.221802699999984pt/>. Therefore what we really computed was <img src="/tex/54379c8d9072543e8a77d2a03cb9cd46.svg?invert_in_darkmode&sanitize=true" align=middle width=33.06963165pt height=28.92634470000001pt/>. Well that's lucky, otherwise I would've just wasted a whole bunch of time. 

The final piece of the puzzle is the meaning of <img src="/tex/29d0c0d542875a3ee7d83e4ed26a5a21.svg?invert_in_darkmode&sanitize=true" align=middle width=30.70407449999999pt height=24.65753399999998pt/>. Let's start by figuring out what units <img src="/tex/29d0c0d542875a3ee7d83e4ed26a5a21.svg?invert_in_darkmode&sanitize=true" align=middle width=30.70407449999999pt height=24.65753399999998pt/> is in. When we perturb the rotation matrix we compute <img src="/tex/08e7bfe805788b5c0fd2bcdaf15865f0.svg?invert_in_darkmode&sanitize=true" align=middle width=76.30125359999998pt height=24.65753399999998pt/>. If <img src="/tex/4f4f4e395762a3af4575de74c019ebb5.svg?invert_in_darkmode&sanitize=true" align=middle width=5.936097749999991pt height=20.221802699999984pt/> is in seconds, then <img src="/tex/cf6790d96b3a957f95886f2277062e8b.svg?invert_in_darkmode&sanitize=true" align=middle width=30.61648919999999pt height=24.65753399999998pt/> is <img src="/tex/614ce8a3c43e0ae0a36925a3ed24af9a.svg?invert_in_darkmode&sanitize=true" align=middle width=76.32983985pt height=30.648287999999997pt/>. Which means its a velocity. Wow, that is really quite useful. Let's see if we can pin down what type of velocity it is. To do this, we need to recognize that a <img src="/tex/53e14fe4f3521c64c328f4a15bffeef3.svg?invert_in_darkmode&sanitize=true" align=middle width=36.52961069999999pt height=21.18721440000001pt/> skew-symmetric matrix multiplying a vector is really encoding a [cross-product](https://en.wikipedia.org/wiki/Cross_product). So 

<p align="center"><img src="/tex/bb0bf93ca38549c472f54ed97605d3cd.svg?invert_in_darkmode&sanitize=true" align=middle width=125.60849234999998pt height=16.438356pt/></p>

If you remember your high-school physics (and I certainly do not), you will recall that some velocity, crossed with a position in space is an [angular velocity](https://en.wikipedia.org/wiki/Angular_velocity), the rate of rotation of a point around the origin of a space. Let's use <img src="/tex/9432d83304c1eb0dcb05f092d30a767f.svg?invert_in_darkmode&sanitize=true" align=middle width=11.87217899999999pt height=22.465723500000017pt/> to represent the angular velocity vector and let's start doing that now: <img src="/tex/2fd29476f98bf7a3f3ef3af8c38af917.svg?invert_in_darkmode&sanitize=true" align=middle width=175.863105pt height=24.65753399999998pt/>. 

There's one last missing piece, which is our rotation matrix <img src="/tex/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode&sanitize=true" align=middle width=12.60847334999999pt height=22.465723500000017pt/>. One of the hardest things to keep straight in rigid body mechanics is in what space quantities are defined in. Remember, that <img src="/tex/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode&sanitize=true" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is  mapping vectors between the undeformed (often called the body space) . Normally these vectors are position vectors (the vector from the origin of a space to a point in that same space). But they can also be other things, like velocities! Remember our time derivative is <img src="/tex/b188c6023033143e829ccdb3f790a455.svg?invert_in_darkmode&sanitize=true" align=middle width=53.384558699999985pt height=24.65753399999998pt/> -- that rotation matrix is mapping the angular velocity, <img src="/tex/ca18e05f77406311d7f05901cda73235.svg?invert_in_darkmode&sanitize=true" align=middle width=21.00462704999999pt height=24.65753399999998pt/>, from the undeformed space to the world. This means that <img src="/tex/9432d83304c1eb0dcb05f092d30a767f.svg?invert_in_darkmode&sanitize=true" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is defined in the undeformed space. Now, some people will leave things at that, but I find it a bit weird, especially since in all the previous assignments our velocities are defined in the world space. So I'm going to define a world space angular velocity, <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/>, so that <img src="/tex/5f2b157de7775333bba680a29625c70c.svg?invert_in_darkmode&sanitize=true" align=middle width=67.57577805pt height=27.6567522pt/>. After all this we arrive at 

<p align="center"><img src="/tex/9921eafd49b51774cf26fcdff2b0461a.svg?invert_in_darkmode&sanitize=true" align=middle width=154.1923779pt height=20.2374942pt/></p>

where the final rearrangement (swapping <img src="/tex/e282a6bc7b7541b024887386e6f8f329.svg?invert_in_darkmode&sanitize=true" align=middle width=33.78598739999999pt height=27.6567522pt/> and <img src="/tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode&sanitize=true" align=middle width=14.29216634999999pt height=22.55708729999998pt/>) exploits the properties of skew symmetric matrices. Now we can finally chose <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> and <img src="/tex/98950719c57ad1365b77188f467ad74a.svg?invert_in_darkmode&sanitize=true" align=middle width=10.502226899999991pt height=22.41366929999999pt/> as our generalized velocities. As is typical, we would like to rewrite this in matrix form, and so it becomes

<p align="center"><img src="/tex/206c0b39c65330d028f494e34f5ea7e8.svg?invert_in_darkmode&sanitize=true" align=middle width=185.58550065pt height=64.16471159999999pt/></p>

where <img src="/tex/407e460dcae5ff3f801915b3f0b5eb83.svg?invert_in_darkmode&sanitize=true" align=middle width=44.81722409999999pt height=24.65753399999998pt/> is the *rigid body jacobian* (sometimes written as <img src="/tex/b2af456716f3117a91da7afe70758041.svg?invert_in_darkmode&sanitize=true" align=middle width=10.274003849999989pt height=22.465723500000017pt/>). 

**Important Note:** matrix exponentials can be expensive to compute. However, for the special orthogonal group, in 3-dimensions, there's a beautiful analytical short cut called [Rodrigues' Rotation Formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula) (note the "s"). You will implement this for the assignment.

## Kinetic energy

We finally reach a point where the elegance of the variational approach starts to kick in. As usual we define kinetic energy as 

<p align="center"><img src="/tex/05010bb9e01206005a585fb61e12b30b.svg?invert_in_darkmode&sanitize=true" align=middle width=272.82351194999995pt height=40.5483111pt/></p>

By direct substitution we end up with

<p align="center"><img src="/tex/714b68c0dec09009083d9d4afc5ee1d1.svg?invert_in_darkmode&sanitize=true" align=middle width=547.7929281pt height=122.13761174999999pt/></p>

The bad news is that <img src="/tex/fb97d38bcc19230b0acd442e17db879c.svg?invert_in_darkmode&sanitize=true" align=middle width=17.73973739999999pt height=22.465723500000017pt/> is no longer constant, it depends on the orientation of the rigid body, but the partial good news, is that a big chunk of it, <img src="/tex/7ae1d6da5db0b75a96bddc0e0fc2ab9a.svg?invert_in_darkmode&sanitize=true" align=middle width=22.500061649999992pt height=22.465723500000017pt/> can be precomputed. Plus, we can make our lives a bit easier by looking at some individual components of <img src="/tex/7ae1d6da5db0b75a96bddc0e0fc2ab9a.svg?invert_in_darkmode&sanitize=true" align=middle width=22.500061649999992pt height=22.465723500000017pt/>. 

First, note that the lower-right block of <img src="/tex/7ae1d6da5db0b75a96bddc0e0fc2ab9a.svg?invert_in_darkmode&sanitize=true" align=middle width=22.500061649999992pt height=22.465723500000017pt/> is <img src="/tex/9f879f53649a8f044743747862099316.svg?invert_in_darkmode&sanitize=true" align=middle width=95.47943459999998pt height=26.48417309999999pt/>. This integral is trivially equal to <img src="/tex/a9a2c021f7887c4338c06c1648a14c6c.svg?invert_in_darkmode&sanitize=true" align=middle width=22.949087699999986pt height=22.465723500000017pt/> where <img src="/tex/0e51a2dede42189d77627c4d742822c3.svg?invert_in_darkmode&sanitize=true" align=middle width=14.433101099999991pt height=14.15524440000002pt/> is the mass of the entire object. 

Second, let's consider the off-diagonal blocks, of the form <img src="/tex/637ea435250c51ab38bcd8cf5d7c822a.svg?invert_in_darkmode&sanitize=true" align=middle width=115.86739395pt height=26.48417309999999pt/> (or the transpose). Remember that each entry of <img src="/tex/2645f0dcd5b25f1bd8b57f2e6e47e8cb.svg?invert_in_darkmode&sanitize=true" align=middle width=23.42461439999999pt height=24.65753399999998pt/>  is just a component (either <img src="/tex/cbfb1b2a33b28eab8a3e59464768e810.svg?invert_in_darkmode&sanitize=true" align=middle width=14.908688849999992pt height=22.465723500000017pt/>, <img src="/tex/91aac9730317276af725abd8cef04ca9.svg?invert_in_darkmode&sanitize=true" align=middle width=13.19638649999999pt height=22.465723500000017pt/>, <img src="/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/>) of <img src="/tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode&sanitize=true" align=middle width=14.29216634999999pt height=22.55708729999998pt/>. So the entries of this matrix are one of <img src="/tex/81e1c86713a48f671998fc406a43c111.svg?invert_in_darkmode&sanitize=true" align=middle width=146.872044pt height=26.48417309999999pt/>, <img src="/tex/297700b33ec7c1ae3740413514d063c6.svg?invert_in_darkmode&sanitize=true" align=middle width=142.64838719999997pt height=26.48417309999999pt/>, <img src="/tex/23bf3ad5ca5106dd2778895f35666fd9.svg?invert_in_darkmode&sanitize=true" align=middle width=138.42459014999997pt height=26.48417309999999pt/> (or their negations). 

What's interesting is that the vector <img src="/tex/97469d2d8011b7a082b2b5736b5049de.svg?invert_in_darkmode&sanitize=true" align=middle width=135.65977589999997pt height=35.5436301pt/> is the [center-of-mass](https://en.wikipedia.org/wiki/Center_of_mass) of the object. All this time I've been using <img src="/tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode&sanitize=true" align=middle width=14.29216634999999pt height=22.55708729999998pt/> to represent a point in the undeformed space of a rigid body -- and I **never** chose the origin of the space (how naughty of me). Well, now I'm going to make a choice, one which will make my life a lot easier going forward. I'm going to choose the origin of the undeformed space to be **the center-of-mass**. This means that by definition <img src="/tex/c3d6fe7bd0ff303f94f87090d95385c6.svg?invert_in_darkmode&sanitize=true" align=middle width=166.61848829999997pt height=35.5436301pt/>. Because <img src="/tex/0e51a2dede42189d77627c4d742822c3.svg?invert_in_darkmode&sanitize=true" align=middle width=14.433101099999991pt height=14.15524440000002pt/> is greater than zero by definition, the off-diagonal blocks of <img src="/tex/7ae1d6da5db0b75a96bddc0e0fc2ab9a.svg?invert_in_darkmode&sanitize=true" align=middle width=22.500061649999992pt height=22.465723500000017pt/> become **zero**. 

This choice of origin also gives our generalized coordinates and velocities more meaning. Our rotation and translation variables are really measuring rotation around, and translation of, the center-of-mass of our object. 

After all this we get a drastically nicer version of <img src="/tex/7ae1d6da5db0b75a96bddc0e0fc2ab9a.svg?invert_in_darkmode&sanitize=true" align=middle width=22.500061649999992pt height=22.465723500000017pt/> which is 

<p align="center"><img src="/tex/5c6599783d62406c11b949b3859e8766.svg?invert_in_darkmode&sanitize=true" align=middle width=256.1924046pt height=49.315569599999996pt/></p>

This version of <img src="/tex/7ae1d6da5db0b75a96bddc0e0fc2ab9a.svg?invert_in_darkmode&sanitize=true" align=middle width=22.500061649999992pt height=22.465723500000017pt/> has two convenient properties. First, it completely decouples the effect of angular and linear velocities on kinetic energy (this will make the equations of motion nicer). Second, we only have one tricky integral to evaluate. 

One way you could compute the remaining integrals (upper left block and mass) would be to tetrahedralize your simulation mesh and use quadrature. That's a bit unsatisfying because it adds an extra layer of [geometry processing](https://github.com/alecjacobson/geometry-processing-csc2520) to the proceedings (and who wants that ?). For this assignment we are going to do something somewhat more satisfying ... 

### Surface-Only Integration

The method we will use for integration was popularized Brian Mirtich [here](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.56.127&rep=rep1&type=pdf). It uses the [divergence theorem](https://en.wikipedia.org/wiki/Divergence_theorem) to convert volume integrals into surface integrals and thus allows there evaluation using a surface, rather than a volumetric discretization.  The basic idea is to rephrase integrals of the type 

<p align="center"><img src="/tex/742003e74795c9e67e99a0f790a9da95.svg?invert_in_darkmode&sanitize=true" align=middle width=129.0581556pt height=37.3519608pt/></p>

to 

<p align="center"><img src="/tex/634327b35ac4479d579e5af6ea1c8426.svg?invert_in_darkmode&sanitize=true" align=middle width=323.80053749999996pt height=37.3519608pt/></p>

where <img src="/tex/ef19c9d325bb82efdefbcf83571a773a.svg?invert_in_darkmode&sanitize=true" align=middle width=126.65477054999998pt height=24.65753399999998pt/> and <img src="/tex/b56595d2a30a0af329086562ca12d521.svg?invert_in_darkmode&sanitize=true" align=middle width=10.502226899999991pt height=14.611878600000017pt/> is the outward facing surface normal at <img src="/tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode&sanitize=true" align=middle width=14.29216634999999pt height=22.55708729999998pt/>, <img src="/tex/22fa7a42465ead19a178d08c712776d8.svg?invert_in_darkmode&sanitize=true" align=middle width=49.771515749999985pt height=24.65753399999998pt/>.

What makes this particularly attractive for rigid body inertia tensors is that <img src="/tex/e2b98e7767a929414a7aad896a0a07d2.svg?invert_in_darkmode&sanitize=true" align=middle width=39.63466814999999pt height=24.65753399999998pt/> is very simple. For instance, for the mass calculation we have  <img src="/tex/3a300b17048ca3d1c9674065b5c70e66.svg?invert_in_darkmode&sanitize=true" align=middle width=141.12244245pt height=26.48417309999999pt/>. Let's make an easy choice for the function <img src="/tex/e7d302064d022ad2869b42ac6776acc2.svg?invert_in_darkmode&sanitize=true" align=middle width=139.3719162pt height=27.94539330000001pt/>. Therefore our integral becomes

<img src="/tex/d329a14641df1f2506010471a9832fc3.svg?invert_in_darkmode&sanitize=true" align=middle width=176.59409789999998pt height=26.48417309999999pt/>

where <img src="/tex/322d8f61a96f4dd07a0c599482268dfe.svg?invert_in_darkmode&sanitize=true" align=middle width=17.32124954999999pt height=14.15524440000002pt/> is the <img src="/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> component of the surface normal. Because our surfaces are divided up into triangle meshes, we can evaluate this integral one triangle at a time and add up the results which gives - 

<p align="center"><img src="/tex/fbdfd30ba85b9839698d44ad53f970c2.svg?invert_in_darkmode&sanitize=true" align=middle width=197.25350865pt height=49.9887465pt/></p> 

Additionally, because the entries of <img src="/tex/052c1283a82cbc1b97bf6a9fbaa7aa42.svg?invert_in_darkmode&sanitize=true" align=middle width=59.122592099999984pt height=32.255810399999994pt/> are all quadratic in <img src="/tex/d05b996d2c08252f77613c25205a0f04.svg?invert_in_darkmode&sanitize=true" align=middle width=14.29216634999999pt height=22.55708729999998pt/>, similar formulas can be found and applied (Mirtich kindly lists them all in his paper).

## Potential Energy

Since rigid bodies don't deform, they don't store any potential energy. Consider this the return you get for persevering through the rotation stuff :)

## The Equations of Motion

The slightly funny form of the kinetic energy leads to a different set of equations of motion for rigid body simulations. These equations are called [Euler's equations of rigid motion](https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)). They are also the Euler-Lagrange equations for the Principle of Least Action, derived using our rotations and angular velocities as generalized coordinates and velocities. There is a very detailed write up of how this is done [here](http://www.math.ucsd.edu/~mleok/pdf/samplechap.pdf).

As is the usual case we can form the Lagrangian <img src="/tex/4dcc020ed1f079dc660f39680599c075.svg?invert_in_darkmode&sanitize=true" align=middle width=78.32741234999999pt height=22.465723500000017pt/> where <img src="/tex/80a683b80a16a890a4239aea30cacf14.svg?invert_in_darkmode&sanitize=true" align=middle width=43.37887124999999pt height=22.465723500000017pt/> for rigid bodies. Initially it would seem to make sense to use our kinetic energy <img src="/tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode&sanitize=true" align=middle width=11.889314249999991pt height=22.465723500000017pt/> from above, which is parameterized by, <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/>, the world space angular velocity. Sadly, this will make the derivation very difficult, and here's why. Recall that 

<p align="center"><img src="/tex/da53e6320782211f81086c6ea980ee5a.svg?invert_in_darkmode&sanitize=true" align=middle width=57.22018665pt height=11.232861749999998pt/></p>

where <img src="/tex/9432d83304c1eb0dcb05f092d30a767f.svg?invert_in_darkmode&sanitize=true" align=middle width=11.87217899999999pt height=22.465723500000017pt/> is the world space angular velocity. A small variation to <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> can be constructed by varying <img src="/tex/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode&sanitize=true" align=middle width=12.60847334999999pt height=22.465723500000017pt/> and <img src="/tex/9432d83304c1eb0dcb05f092d30a767f.svg?invert_in_darkmode&sanitize=true" align=middle width=11.87217899999999pt height=22.465723500000017pt/> together. This is tricky to account for. Rather than deal with things this way, its is easier to work, for a moment, using <img src="/tex/9432d83304c1eb0dcb05f092d30a767f.svg?invert_in_darkmode&sanitize=true" align=middle width=11.87217899999999pt height=22.465723500000017pt/>. Now our kinetic energy becomes

<p align="center"><img src="/tex/55aac8d63a95d3cf9e6226776816df67.svg?invert_in_darkmode&sanitize=true" align=middle width=164.61835334999998pt height=57.0723219pt/></p>

where <img src="/tex/4b35a2a850babd96154af15a510fd024.svg?invert_in_darkmode&sanitize=true" align=middle width=10.164453749999991pt height=22.465723500000017pt/> is the upper-left block of <img src="/tex/7ae1d6da5db0b75a96bddc0e0fc2ab9a.svg?invert_in_darkmode&sanitize=true" align=middle width=22.500061649999992pt height=22.465723500000017pt/>. I've written it out like this because, for this decoupled system we can apply calculus of variations seperately for the rotational and linear parts. Thus we get two sets of equations of motion, one computed by setting <img src="/tex/a1b1b10a8a84296232f4bdf1a613916b.svg?invert_in_darkmode&sanitize=true" align=middle width=55.04555759999999pt height=22.831056599999986pt/> and the other computed by setting <img src="/tex/26689fc62e68ea825a433505c9a17dcd.svg?invert_in_darkmode&sanitize=true" align=middle width=55.04555759999999pt height=22.831056599999986pt/>. The second equation uses standard, linear velocities and is handled as usual, leading to 

<p align="center"><img src="/tex/1d219cc370e4247d6e709fb17933496d.svg?invert_in_darkmode&sanitize=true" align=middle width=83.53504994999999pt height=14.611878599999999pt/></p>

Here I've added an external forcing term and we observe that the center-of-mass of the rigid body behaves exactly like a regular particle in 3d. 

The rotational component is a little bit tricker because we need to compute <img src="/tex/f3939b6971df726d6ffe9da58149c5cc.svg?invert_in_darkmode&sanitize=true" align=middle width=92.47941119999999pt height=29.46111299999998pt/>. Much like our previous rotational time derivative, <img src="/tex/0d5b5eb0b1fb426ac2db5a4aded9f54b.svg?invert_in_darkmode&sanitize=true" align=middle width=19.800250799999986pt height=22.831056599999986pt/> must take into account the special structure of the Orthogonal Group. Suppressing all the details, this leads to an extra term in the final equations of motion, known as the *Quadratic Velocity Vector*. This gives us the equations of motion for the rotational variables as 

<p align="center"><img src="/tex/47436f526b9cf078f22b2c627c7844ca.svg?invert_in_darkmode&sanitize=true" align=middle width=156.67401255pt height=19.24333455pt/></p>

where <img src="/tex/f6320313c3a5e1c751a07c8a37f357d2.svg?invert_in_darkmode&sanitize=true" align=middle width=25.843189349999992pt height=14.15524440000002pt/> is an external torque applied to the system. This is equivalent to to 

<p align="center"><img src="/tex/b928b8b83a5ce1527d125094b632235f.svg?invert_in_darkmode&sanitize=true" align=middle width=226.95140324999997pt height=20.403824099999998pt/></p>

where <img src="/tex/2534f7925c189ee4e7d1f9084ac73bd6.svg?invert_in_darkmode&sanitize=true" align=middle width=25.843189349999992pt height=14.15524440000002pt/> becomes the world space external torque.

Now all that remains is to integrate our center-of-mass and angular acceleration equations to produce rigid body motion.

## Time Integration of Rotating Objects

Because we have no elastic forces to worry about, we can get away with simpler, explicit time integration (**at least for rigid objects that aren't spinning too quickly**). As such we will apply an explicit Euler type scheme that works in the following way. Like symplectic Euler, we will first compute new velocities for our objects, and then update their positions. For the center-of-mass (particle) equation, this is done using [symplectic Euler](https://github.com/dilevin/CSC2549-a1-mass-spring-1d), exactly!

To update our angular velocities we can proceed as normal, by which I mean replacing our accelerations with standard first order finite differences. Why is this ok for angular accelerations and velocities ? Because these terms act in relation to the tangent space of our Lie Group. The tangent space is a locally flat space (like Euclidean space) and so we can (for a brief moment) ignore all the difficulties rotations and their orthogonality constraint introduce. This means the first step of our integrator solves

<p align="center"><img src="/tex/4a0d826807c4c310394424a4c85566cf.svg?invert_in_darkmode&sanitize=true" align=middle width=455.79338144999997pt height=29.58934275pt/></p>

This is an explicit integration step, we evaluate all the positional variables and forces at the current time step. 

The final tricky part is to update our rotation matrix. Now all the complications return. We can't just add our new angular velocity to the exiting rotation matrix. Recall from our initial discussion of rotations that the angular velocity equation is <img src="/tex/23f4101d83e9be7139ec3f32fd1fd8a7.svg?invert_in_darkmode&sanitize=true" align=middle width=91.56370244999998pt height=27.94539330000001pt/>. If <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> is constant, this is a linear ordinary differential equation and can be solved with (you guessed it) the matrix exponential, yielding

<img src="/tex/c04e2014df095ee2043324f2c1c4fd8e.svg?invert_in_darkmode&sanitize=true" align=middle width=411.1813332pt height=27.94539330000001pt/>

which gives us the updated rotation matrix <img src="/tex/00f5bd6134bc5e5a4bf059a551556ce5.svg?invert_in_darkmode&sanitize=true" align=middle width=195.9134298pt height=27.94539330000001pt/>. 

It's this set of update equations you will use to implement rigid body dynamics in this assignment.

## Assignment Implementation

In this assignment you will implement everything needed to simulate an unconstrained rigid body in free space. This includes the mass matrix integration and the explicit, exponential Euler time integrator. While you are encouraged to consult any linked resources, **DO NOT** use any available source code in your assignment. You must code everything yourself. 

### Implementation Notes

Because the generalized coordinates are no longer just a stacked vector of points, accessing them becomes trickier. To make this a little easier we will use the [``Eigen::Map``](https://eigen.tuxfamily.org/dox/classEigen_1_1Map.html) functionality. This let's you create a proxy linear algebra operator from a raw chunk of memory. In this assignment, the rotation matrix and position vector representing a rigid bodies configuration are flattened out and stored in a single *Eigen VectorXd*. To extract the rotation matrix from the rigid body indexed by ``irb`` you should do the following

``Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix3d>(q.segment<9>(12*irb).data());``

Note that I am using the templated version of the segment method, where the segment size is inside the ``<>``. 

### rodrigues.cpp

The rodrigues formula for computing the marix exponential of a <img src="/tex/46e42d6ebfb1f8b50fe3a47153d01cd2.svg?invert_in_darkmode&sanitize=true" align=middle width=36.52961069999999pt height=21.18721440000001pt/>, skew-symmetric matrix.

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
