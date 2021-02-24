using System;
using System.Collections.Generic;
using System.Numerics;

namespace BaseSLAM
{
    /// <summary>
    /// Scan points cloud
    /// </summary>
    public class ScanCloud
    {
        /// <summary>
        /// Pose at the moment of scanning
        /// </summary>
        public Vector3 Pose { get; set; } = Vector3.Zero;

        /// <summary>
        /// Scan points
        /// </summary>
        public List<Vector2> Points { get; } = new List<Vector2>();
    }
}
