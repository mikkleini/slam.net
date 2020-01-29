using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
using HectorSLAM.Scan;
using HectorSLAM.Util;

namespace HectorSLAM.Main
{
    public class MapRepMultiMap : IMapRepresentation
    {
        private readonly List<MapProcContainer> mapContainer;
        private readonly List<DataContainer> dataContainers;

        public float ScaleToMap => mapContainer[0].GridMap.ScaleToMap;
        public int MapLevels => mapContainer.Count;

        public MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY, int numDepth, Vector2 startCoords, IDrawInterface drawInterface, IHectorDebugInfo debugInterface)
        {
            //unsigned int numDepth = 3;
            Point resolution = new Point(mapSizeX, mapSizeY);

            float totalMapSizeX = mapResolution * mapSizeX;
            float mid_offset_x = totalMapSizeX * startCoords.X;

            float totalMapSizeY = mapResolution * mapSizeY;
            float mid_offset_y = totalMapSizeY * startCoords.Y;

            mapContainer = new List<MapProcContainer>();
            dataContainers = new List<DataContainer>(numDepth - 1);

            for (int i = 0; i < numDepth; ++i)
            {
                Console.WriteLine($"HectorSM map lvl {i}: cellLength: {mapResolution} res x: {resolution.X} res y: {resolution.Y}");
                GridMap gridMap = new GridMap(mapResolution, resolution, new Vector2(mid_offset_x, mid_offset_y));
                OccGridMapUtilConfig gridMapUtil = new OccGridMapUtilConfig(gridMap);
                ScanMatcher scanMatcher = new ScanMatcher(drawInterface, debugInterface);

                mapContainer.Add(new MapProcContainer(gridMap, gridMapUtil, scanMatcher));

                resolution = new Point(resolution.X / 2, resolution.Y / 2);
                mapResolution *= 2.0f;
            }
        }

        public void Reset()
        {
            mapContainer.ForEach(m => m.Reset());
        }

        public GridMap GetGridMap(int mapLevel = 0)
        {
            return mapContainer[mapLevel].GridMap;
        }

        public void AddMapMutex(int i, IMapLocker mapMutex)
        {
            mapContainer[i].MapMutex = mapMutex;
        }

        public IMapLocker GetMapMutex(int i)
        {
            return mapContainer[i].MapMutex;
        }

        public void OnMapUpdated()
        {
            mapContainer.ForEach(m => m.ResetCachedData());
        }

        public Vector3 MatchData(Vector3 beginEstimateWorld, DataContainer dataContainer, out Matrix4x4 covMatrix)
        {
            Vector3 tmp = beginEstimateWorld;
            covMatrix = Matrix4x4.Identity; // It should not be returned

            for (int index = mapContainer.Count - 1; index >= 0; --index)
            {
                if (index == 0)
                {
                    tmp = mapContainer[index].MatchData(tmp, dataContainer, out covMatrix, 5);
                }
                else
                {
                    dataContainers[index - 1].SetFrom(dataContainer, 1.0f / MathF.Pow(2.0f, index));
                    tmp = mapContainer[index].MatchData(tmp, dataContainers[index - 1], out covMatrix, 3);
                }
            }

            return tmp;
        }

        public void UpdateByScan(DataContainer dataContainer, Vector3 robotPoseWorld)
        {
            for (int i = 0; i < mapContainer.Count; ++i)
            {
                if (i == 0)
                {
                    mapContainer[i].UpdateByScan(dataContainer, robotPoseWorld);
                }
                else
                {
                    mapContainer[i].UpdateByScan(dataContainers[i - 1], robotPoseWorld);
                }
            }
        }

        public void SetUpdateFactorFree(float factor)
        {
            mapContainer.ForEach(p => p.GridMap.SetUpdateFreeFactor(factor));
        }

        public void SetUpdateFactorOccupied(float factor)
        {
            mapContainer.ForEach(p => p.GridMap.SetUpdateOccupiedFactor(factor));
        }
    }
}
