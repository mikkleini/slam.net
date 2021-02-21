using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;
using System.Linq;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
using HectorSLAM.Scan;
using HectorSLAM.Util;

namespace HectorSLAM.Main
{
    public class MapRepMultiMap : IMapRepresentation
    {
        private readonly MapProcContainer[] mapContainers;
        private readonly DataContainer[] dataContainers;

        public float ScaleToMap => mapContainers[0].GridMap.ScaleToMap;

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
            dataContainers = new DataContainer[numDepth - 1];

            for (int i = 0; i < numDepth - 1; i++)
            {
                dataContainers[i] = new DataContainer();
            }

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

        public Vector3 MatchData(Vector3 beginEstimateWorld, DataContainer dataContainer, out Matrix4x4 covMatrix)
        {
            Vector3 tmp = beginEstimateWorld;
            covMatrix = Matrix4x4.Identity; // It should not be returned

            for (int index = mapContainers.Length - 1; index >= 0; --index)
            {
                if (index == 0)
                {
                    tmp = mapContainers[index].MatchData(tmp, dataContainer, out covMatrix, 5);
                }
                else
                {
                    dataContainers[index - 1].SetFrom(dataContainer, 1.0f / MathF.Pow(2.0f, index));
                    tmp = mapContainers[index].MatchData(tmp, dataContainers[index - 1], out covMatrix, 3);
                }
            }

            return tmp;
        }

        public void UpdateByScan(DataContainer dataContainer, Vector3 robotPoseWorld)
        {
            for (int i = 0; i < mapContainers.Length; ++i)
            {
                if (i == 0)
                {
                    mapContainers[i].UpdateByScan(dataContainer, robotPoseWorld);
                }
                else
                {
                    mapContainers[i].UpdateByScan(dataContainers[i - 1], robotPoseWorld);
                }
            }
        }

        public void SetUpdateFactorFree(float factor)
        {
            mapContainers.ForEach(p => p.GridMap.SetUpdateFreeFactor(factor));
        }

        public void SetUpdateFactorOccupied(float factor)
        {
            mapContainers.ForEach(p => p.GridMap.SetUpdateOccupiedFactor(factor));
        }
    }
}
