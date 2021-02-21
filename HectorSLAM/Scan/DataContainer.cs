using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Scan
{
    /// <summary>
    /// Lidar beam hitpoints container
    /// </summary>
    public class DataContainer : List<Vector2>
    {
        /// <summary>
        /// Lidar rays origin point in meters (zero vector if lidar is in the center of the robot)
        /// </summary>
        public Vector2 Origin { get; set; } = Vector2.Zero;

        /// <summary>
        /// Set from other container
        /// </summary>
        /// <param name="other">Other data container</param>
        /// <param name="factor">Scaling factor</param>
        public void SetFrom(DataContainer other, float factor)
        {
            Origin = other.Origin * factor;

            Clear();

            foreach (Vector2 v in other)
            {
                Add(v * factor);
            }
        }
    }
}
