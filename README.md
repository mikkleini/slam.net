# slam.net
Simultaneous localization and mapping libraries for C#

Here are two C# SLAM implementations: CoreSLAM and HectorSLAM. Both use base classes from BaseSLAM library.

*CoreSLAM*

This is based on the CoreSLAM algorithm by Bruno Steux, Oussama El Hamzaoui.
The Monte-carlo search function has been looked at some other C and C++ algorithms but I did differently.
First of all there's always fixed number of iterations to get predictible speed.
Additionally the search run on multiple threads to get get maximum performance on multi-core platform.
It also uses [Redzen](https://www.nuget.org/packages/Redzen) fast normal distribution random number generator.

I used it in my Robotex 2019 robotic competition robot "Ace Venture" and won the "Starship animal rescure" competition.
I used which Raspberry Pi 4 with .NET Core 3.0, but it also works on Windows.
To speed it up even further it should be possible to use ARM SIMD instruction set NEON, if .NET Core devs implement ARM intrinsics.

![Demo map][demo_map.png]

*HectorSLAM*
