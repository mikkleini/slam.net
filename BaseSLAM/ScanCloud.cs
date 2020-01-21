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
        /// Scan points
        /// </summary>
        public List<Vector2> Points { get; } = new List<Vector2>();
    }
}
