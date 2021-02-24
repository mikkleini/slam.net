using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
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

        public float ScaleToMap => gridMap.Properties.ScaleToMap;

        public int NumLevels => 1;

        public GridMap GetGridMap(int mapLevel)
        {
            return gridMap;
        }

        public void OnMapUpdated()
        {
            gridMapUtil.ResetCachedData();
        }

        public Vector3 MatchData(Vector3 beginEstimateWorld, ScanCloud scan, out Matrix4x4 covMatrix)
        {
            return scanMatcher.MatchData(beginEstimateWorld, gridMapUtil, scan, out covMatrix, 20);
        }

        public void UpdateByScan(ScanCloud scan, Vector3 robotPoseWorld)
        {
            gridMap.UpdateByScan(scan, robotPoseWorld);
        }

        public void AddMapMutex(int i, IMapLocker mapMutex)
        {
            throw new NotImplementedException();
        }

        public IMapLocker GetMapMutex(int i)
        {
            throw new NotImplementedException();
        }

        public void SetUpdateFactorFree(float factor)
        {
            throw new NotImplementedException();
        }

        public void SetUpdateFactorOccupied(float factor)
        {
            throw new NotImplementedException();
        }
    }
}
