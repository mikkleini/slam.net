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
It ran on Raspberry Pi 4 using .NET Core 3.0 and there was some CPU time left over for other tasks also. But now the code is for .NET 5.0.
My problem with this SLAM algorithm was that even with wheel odometry the map tilted and slided over time.
However, it may have been just a configuration issue and for that reason I created a simulation application to quickly try out different parameters (hole width, etc.).

Here's the picture of the CoreSLAM functioning in simulation:
![Simulation](simulation_coreslam.png)

The grayscale map is the hole map. Blue edges are true walls. Blue circle is true position, red is estimated position.
When you compile and run simulation, use left mouse button to drag the real position and see how the hole map estimator works.
If you click further than SigmaXY the estimator may fail to find correct position.
There is artificial noise added to the "lidar" measurements.
Parameters tuning is in the code, no UI features for that.

The lidar ray tracing is done with .NET variant of [Box2D](https://github.com/benukhanov/box2d-netstandard).

## HectorSLAM

This is the port of [HectorSLAM](https://github.com/tu-darmstadt-ros-pkg/hector_slam) algorithm from C++ to C#.
HectorSLAM is originally a ROS package and based on what I've read and seen it looks much more stable than CoreSLAM.
Since I'm not very fond of C++ boilerplate code (getters and setters) I started porting it and actually it went much smaller in lines of code.
Actually there's quite a bit of unused functions also, so it's possible to strip it down even further.
The code uses .NET System.Numerics namespace so there's no external dependency.

In the simulator it works fine as long as the robot (mouse cursor) movement speed per lidar scan is less than the physical pixels size of coarsest map.
E.g. if the resolution is 0.05m, map size is 400x400, map levels is 3, scan rate is 7 scans/sec then max speed must be:
  (400 * 0.05m) / 2<sup>3</sup> => 2.5 m/scan => 0,35 m/s.
I haven't yet tried in real world.

----

*PS.*

You can get the RPLidar A1 C# code from [here](https://github.com/mikkleini/rplidar.net). To do path planning, you can use my fork of [Roy-T AStar]( https://github.com/mikkleini/AStar). The master AStar library is now very different and due to lack of agent size wouldn't fit.
