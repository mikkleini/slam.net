using System;
using System.Collections.Generic;
using System.Text;

namespace BaseSLAM
{
    /// <summary>
    /// Point in polar coordinate system.
    /// </summary>
    public struct Ray
    {
        /// <summary>
        /// Angle in radians
        /// </summary>
        public float Angle { get; set; }

        /// <summary>
        /// Radius in meters
        /// </summary>
        public float Radius { get; set; }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="angle">Angle in radians</param>
        /// <param name="radius">Radius in meters</param>
        public Ray(float angle, float radius)
        {
            Angle = angle;
            Radius = radius;
        }
    }
}
