using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
using HectorSLAM.Scan;
using HectorSLAM.Util;

namespace HectorSLAM.Main
{
    public class MapProcContainer
    {
        public GridMap GridMap { get; private set; }
        public OccGridMapUtilConfig GridMapUtil { get; private set; }
        public ScanMatcher ScanMatcher { get; private set; }
        public IMapLocker MapMutex { get; set; }

        public MapProcContainer(GridMap gridMap, OccGridMapUtilConfig gridMapUtil, ScanMatcher scanMatcher)
        {
            GridMap = GridMap;
            GridMapUtil = gridMapUtil;
            ScanMatcher = scanMatcher;
            MapMutex = null;
        }

        public void Reset()
        {
            GridMap.Reset();
            GridMapUtil.ResetCachedData();
        }

        public void ResetCachedData()
        {
            GridMapUtil.ResetCachedData();
        }

        public Vector3 MatchData(Vector3 beginEstimateWorld, DataContainer dataContainer, Matrix4x4 covMatrix, int maxIterations)
        {
            return ScanMatcher.MatchData(beginEstimateWorld, GridMapUtil, dataContainer, covMatrix, maxIterations);
        }

        public void UpdateByScan(DataContainer dataContainer, Vector3 robotPoseWorld)
        {
            if (MapMutex != null)
            {
                MapMutex.Lock();
            }

            GridMap.UpdateByScan(dataContainer, robotPoseWorld);

            if (MapMutex != null)
            {
                MapMutex.Unlock();
            }
        }
    }
}
