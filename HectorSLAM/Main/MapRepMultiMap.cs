using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;
using System.Linq;
using System.Threading;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
using HectorSLAM.Util;
using System.Threading.Tasks;
using System.Diagnostics;

namespace HectorSLAM.Main
{
    public class MapRepMultiMap : IMapRepresentation
    {
        private readonly MapProcContainer[] mapContainers;
        private readonly ParallelOptions parallelOptions;

        public float ScaleToMap => mapContainers[0].GridMap.Properties.ScaleToMap;

        /// <summary>
        /// Number of map levels
        /// </summary>
        public int NumLevels => mapContainers.Length;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="mapResolution">Meters per pixel</param>
        /// <param name="mapSizeX">Size X in pixels</param>
        /// <param name="mapSizeY">Size Y in pixels</param>
        /// <param name="numDepth">Number of map levels</param>
        /// <param name="startCoords">Start coordinates as fraction of whole map size</param>
        /// <param name="drawInterface"></param>
        /// <param name="debugInterface"></param>
        public MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY, int numDepth, Vector2 startCoords, IDrawInterface drawInterface, IHectorDebugInfo debugInterface)
        {
            Point resolution = new Point(mapSizeX, mapSizeY);

            float totalMapSizeX = mapResolution * mapSizeX;     // Meters
            float mid_offset_x = totalMapSizeX * startCoords.X; // Meters

            float totalMapSizeY = mapResolution * mapSizeY;
            float mid_offset_y = totalMapSizeY * startCoords.Y;

            mapContainers = new MapProcContainer[numDepth];
            parallelOptions = new ParallelOptions()
            {
                MaxDegreeOfParallelism = numDepth
            };

            for (int i = 0; i < numDepth; ++i)
            {
                System.Diagnostics.Debug.WriteLine($"HectorSM map level {i}: cellLength: {mapResolution} res x: {resolution.X} res y: {resolution.Y}");
                GridMap gridMap = new GridMap(mapResolution, resolution, new Vector2(mid_offset_x, mid_offset_y));
                OccGridMapUtilConfig gridMapUtil = new OccGridMapUtilConfig(gridMap);
                ScanMatcher scanMatcher = new ScanMatcher(drawInterface, debugInterface);

                mapContainers[i] = new MapProcContainer(gridMap, gridMapUtil, scanMatcher);

                // Decrease map accuracy - next level is half the pixel size of previous level
                resolution = new Point(resolution.X / 2, resolution.Y / 2);
                mapResolution *= 2.0f;
            }
        }

        public void Reset()
        {
            mapContainers.ForEach(m => m.Reset());
        }

        public GridMap GetGridMap(int mapLevel = 0)
        {
            return mapContainers[mapLevel].GridMap;
        }

        public void AddMapMutex(int i, IMapLocker mapMutex)
        {
            mapContainers[i].MapMutex = mapMutex;
        }

        public IMapLocker GetMapMutex(int i)
        {
            return mapContainers[i].MapMutex;
        }

        public void OnMapUpdated()
        {
            mapContainers.ForEach(m => m.ResetCachedData());
        }

        /// <summary>
        /// Match data and get best position estimate
        /// </summary>
        /// <param name="beginEstimate">Estimation hint position in world coordinates (meters)</param>
        /// <param name="scan">Scanned points</param>
        /// <param name="covMatrix"></param>
        /// <returns></returns>
        public Vector3 MatchData(Vector3 beginEstimate, ScanCloud scan, out Matrix4x4 covMatrix)
        {
            Vector3 estimate = beginEstimate;
            covMatrix = Matrix4x4.Identity; // It should not be returned

            //System.Diagnostics.Debug.WriteLine($"  Match start at pose {tmp}");

            // Start matching from coarsest map
            for (int index = mapContainers.Length - 1; index >= 0; --index)
            {
                estimate = mapContainers[index].MatchData(estimate, scan, out covMatrix, (index == 0 ? 5 : 3));

                //System.Diagnostics.Debug.WriteLine($"  Match level {index} pose {tmp}");
            }

            return estimate;
        }

        /// <summary>
        /// Update map layers
        /// </summary>
        /// <param name="scan">Scanned points</param>
        /// <param name="robotPoseWorld">Robot pose in world coordinates (meters)</param>
        public void UpdateByScan(ScanCloud scan, Vector3 robotPoseWorld)
        {
            // Run update in parallel tasks. It's about twice as fast on 6 core CPU.
            Parallel.ForEach(mapContainers, parallelOptions, m => m.UpdateByScan(scan, robotPoseWorld));
        }

        /// <summary>
        /// Set update factor free on all map layers
        /// </summary>
        /// <param name="factor">Factor from 0 to 1</param>
        public void SetUpdateFactorFree(float factor)
        {
            mapContainers.ForEach(m => m.GridMap.SetUpdateFreeFactor(factor));
        }

        /// <summary>
        /// Set update factor occupied on all map layers
        /// </summary>
        /// <param name="factor">Factor from 0 to 1</param>
        public void SetUpdateFactorOccupied(float factor)
        {
            mapContainers.ForEach(m => m.GridMap.SetUpdateOccupiedFactor(factor));
        }
    }
}
