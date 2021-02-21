using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    public class GridMap : OccGridMapBase<LogOddsCell>
    {
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="mapResolution">Map resolution in meters per pixel</param>
        /// <param name="size">Map size in pixels</param>
        /// <param name="offset">Offset if meters</param>
        public GridMap(float mapResolution, Point size, Vector2 offset)
            : base(mapResolution, size, offset)
        {
        }
    }
}
