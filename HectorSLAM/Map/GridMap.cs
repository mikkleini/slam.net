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
        /// <param name="mapResolution">Map resolution</param>
        /// <param name="size">Map size</param>
        /// <param name="offset">Offset</param>
        public GridMap(float mapResolution, Point size, Vector2 offset)
            : base(mapResolution, size, offset)
        {
        }
    }
}
