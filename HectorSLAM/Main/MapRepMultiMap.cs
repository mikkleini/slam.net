using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Diagnostics;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;

namespace HectorSLAM.Main
{
    /// <summary>
    /// Multi-level map
    /// </summary>
    public class MapRepMultiMap
    {
        private readonly ParallelOptions parallelOptions;

        /// <summary>
        /// Number of map levels
        /// </summary>
        public int NumLevels => Maps.Length;
        
        /// <summary>
        /// Grid maps.
        /// </summary>
        public OccGridMap[] Maps { get; }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="mapResolution">Meters per pixel</param>
        /// <param name="mapSize">Map size pixels</param>
        /// <param name="numDepth">Number of map levels</param>
        /// <param name="startCoords">Start coordinates as fraction of whole map size</param>
        public MapRepMultiMap(float mapResolution, Point mapSize, int numDepth, Vector2 startCoords)
        {
            parallelOptions = new ParallelOptions()
            {
                MaxDegreeOfParallelism = numDepth
            };

            // Create map
            Maps = new OccGridMap[numDepth];
            for (int i = 0; i < numDepth; ++i)
            {
                Debug.WriteLine($"HectorSLAM map level #{i} resolution: {mapResolution} m/pix, size: {mapSize}");
                Maps[i] = new OccGridMap(mapResolution, mapSize, startCoords);

                // Decrease map accuracy - next level is half the pixel size of previous level
                mapSize = new Point(mapSize.X / 2, mapSize.Y / 2);
                mapResolution *= 2.0f;
            }
        }

        /// <summary>
        /// Reset maps
        /// </summary>
        public void Reset()
        {
            Maps.ForEach(m => m.Reset());
        }

        /// <summary>
        /// Update map layers
        /// </summary>
        /// <param name="scan">Scanned points</param>
        /// <param name="pose">Lidar pose in world coordinates (meters)</param>
        public void UpdateByScan(ScanCloud scan, Vector3 pose)
        {
            // Run update in parallel tasks. It's about twice as fast on 6 core CPU as doing sequentially
            Parallel.ForEach(Maps, parallelOptions, m => m.UpdateByScan(scan, pose));
        }

        /// <summary>
        /// Set update factor free on all map layers
        /// </summary>
        /// <param name="factor">Factor from 0 to 1</param>
        public void SetUpdateFactorFree(float factor)
        {
            Maps.ForEach(m => m.UpdateFreeFactor = factor);
        }

        /// <summary>
        /// Set update factor occupied on all map layers
        /// </summary>
        /// <param name="factor">Factor from 0 to 1</param>
        public void SetUpdateFactorOccupied(float factor)
        {
            Maps.ForEach(m => m.UpdateOccupiedFactor = factor);
        }
    }
}
