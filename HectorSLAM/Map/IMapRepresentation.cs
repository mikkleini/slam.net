using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    public interface IMapRepresentation
    {
        float ScaleToMap { get; }
        int MapLevels { get; }
        void Reset();
        GridMap GetGridMap(int mapLevel = 0);
        void AddMapMutex(int i, IMapLockerInterface mapMutex);
        IMapLockerInterface GetMapMutex(int i);
        void OnMapUpdated();
        Vector3 MatchData(Vector3 beginEstimateWorld, DataContainer dataContainer, Matrix4x4 covMatrix);
        void UpdateByScan(DataContainer dataContainer, Vector3 robotPoseWorld);
        void SetUpdateFactorFree(float free_factor);
        void SetUpdateFactorOccupied(float occupied_factor);
    }
}
