using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using HectorSLAM.Map;
using HectorSLAM.Scan;
using HectorSLAM.Util;

namespace HectorSLAM.Main
{
    public interface IMapRepresentation
    {
        float ScaleToMap { get; }
        int MapLevels { get; }
        void Reset();
        GridMap GetGridMap(int mapLevel = 0);
        void AddMapMutex(int i, IMapLocker mapMutex);
        IMapLocker GetMapMutex(int i);
        void OnMapUpdated();
        Vector3 MatchData(Vector3 beginEstimateWorld, DataContainer dataContainer, out Matrix4x4 covMatrix);
        void UpdateByScan(DataContainer dataContainer, Vector3 robotPoseWorld);
        void SetUpdateFactorFree(float factor);
        void SetUpdateFactorOccupied(float factor);
    }
}
