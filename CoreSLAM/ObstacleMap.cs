using System;
using System.Collections.Generic;
using System.Text;

namespace CoreSLAM
{
    /// <summary>
    /// Obstacle map
    /// </summary>
    public class ObstacleMap
    {
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="sizePixels">Map size in pixels</param>
        /// <param name="sizeMeters">Map size in meters</param>
        public ObstacleMap(int sizePixels, float sizeMeters)
        {
            Size = sizePixels;
            Scale = sizePixels / sizeMeters;
            Pixels = new sbyte[sizePixels, sizePixels];
        }

        /// <summary>
        /// Pixels in 2D array (first is Y, then X axis)
        /// Values:
        ///   -N  - unmapped. N is number of scans required to consider this pixel as clear or contain something.
        ///    0  - clear
        ///   +N  - contains something. N marks number of lidar hits (is limited).
        /// </summary>
        public readonly sbyte[,] Pixels;

        /// <summary>
        /// Map size in pixels
        /// </summary>
        public int Size { get; }

        /// <summary>
        /// Map scale (pixels per meter)
        /// </summary>
        public float Scale { get; }
    }
}
