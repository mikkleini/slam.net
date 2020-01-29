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
    public class MapRepSingleMap : IMapRepresentation
    {
        protected GridMap gridMap;
        protected OccGridMapUtilConfig gridMapUtil;
        protected ScanMatcher scanMatcher;

        public MapRepSingleMap(float mapResolution, IDrawInterface drawInterface, IHectorDebugInfo debugInterface)
        {
            gridMap = new GridMap(mapResolution, new Point(1024, 1024), new Vector2(20.0f, 20.0f));
            gridMapUtil = new OccGridMapUtilConfig(gridMap);
            scanMatcher = new ScanMatcher(drawInterface, debugInterface);
        }

        public void Reset()
        {
            gridMap.Reset();
            gridMapUtil.ResetCachedData();
        }

        public float ScaleToMap => gridMap.ScaleToMap;

        public int MapLevels => 1;

        public GridMap GetGridMap(int mapLevel)
        {
            return gridMap;
        }

        public void OnMapUpdated()
        {
            gridMapUtil.ResetCachedData();
        }

        public Vector3 MatchData(Vector3 beginEstimateWorld, DataContainer dataContainer, Matrix4x4 covMatrix)
        {
            return scanMatcher.MatchData(beginEstimateWorld, gridMapUtil, dataContainer, covMatrix, 20);
        }

        public void UpdateByScan(DataContainer dataContainer, Vector3 robotPoseWorld)
        {
            gridMap.UpdateByScan(dataContainer, robotPoseWorld);
        }
    }
}
