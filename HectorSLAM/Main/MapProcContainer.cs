using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
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
            GridMap = gridMap;
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

        public Vector3 MatchData(Vector3 beginEstimateWorld, ScanCloud scan, out Matrix4x4 covMatrix, int maxIterations)
        {
            return ScanMatcher.MatchData(beginEstimateWorld, GridMapUtil, scan, out covMatrix, maxIterations);
        }

        public void UpdateByScan(ScanCloud scan, Vector3 robotPoseWorld)
        {
            if (MapMutex != null)
            {
                MapMutex.Lock();
            }

            GridMap.UpdateByScan(scan, robotPoseWorld);

            if (MapMutex != null)
            {
                MapMutex.Unlock();
            }
        }
    }
}
