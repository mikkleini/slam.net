# SLAM.NET

Simultaneous Localization And Mapping (SLAM) libraries for C#

Here are two C# SLAM implementations: CoreSLAM and HectorSLAM. Both use base classes from BaseSLAM library.

## CoreSLAM

This is based on the CoreSLAM algorithm by Bruno Steux, Oussama El Hamzaoui (I can't find an authentic link to it...).
The Monte-carlo search function has been looked from some other C and C++ implementations but I did differently.
First of all there's always fixed number of iterations to get stable speed.
Additionally the search is done on multiple threads to get maximum performance on multi-core platform.
It also uses [Redzen](https://www.nuget.org/packages/Redzen) fast normal distribution random number generator and the trick is to pre-calculate random numbers. So when it's time do localization, the right amount of random numbers are ready to use.

I used it in my [Robotex](https://robotex.international) 2019 robotic competition robot "Ace Ventura" and won the "Starship animal rescure" competition, so it kind of works.
My problem with this SLAM algorithm was that even with wheel odometry the map tilted and slided over time.
However, it may have be just a configuration issue and for that reason i creted a simulation application to try out different parameters (hole width, etc.).

Here's the picture of the CoreSLAM functioning in simulation:
![Simulation](simulation_coreslam.png)
The grayscale map is the hole map. Blue edges are true walls. Blue circle is true position, red is estimated position.
When you compiler and run simulation use left mouse button to drag the real position and see how the estimator works.
There is artificial noise added to the "lidar" measurements.
The lidar ray tracing is done with .NET variant of [Box2D](https://github.com/benukhanov/box2d-netstandard).

## HectorSLAM

This is the attempt to port [HectorSLAM](https://github.com/tu-darmstadt-ros-pkg/hector_slam) algorithm from C++ to C#.
HectorSLAM is a ROS package and based on what I've read and seen it looks much more stable than CoreSLAM.
Since I'm not very fond of C++ boilerplate code (getters and setters) I started porting it and actually it went much smaller in lines of code.
However this is very-very raw and doesn't work. I've just manage to make it compile, but haven't yet tried out.
I wanted to use .NET Core System.Numerics namespace, but the downside is that Numerics doesn't have 3x3 matrixes and other helper functions which are in Eigen C++ library (what the original HectorSLAM uses).

So let's say it's work in progress.

----

*PS.*

You can get the RPLidar A1 C# code from [here](https://github.com/mikkleini/rplidar.net). To do path planning, you can use my fork of [Roy-T AStar]( https://github.com/mikkleini/AStar). The master AStar library is now very different and due to lack of agent size wouldn't fit.
