using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
using HectorSLAM.Scan;
using HectorSLAM.Util;

namespace HectorSLAM.Main
{
    public class HectorSLAMProcessor
    {
        private readonly IDrawInterface drawInterface;
        private readonly IHectorDebugInfo debugInterface;

        public MapRepMultiMap MapRep { get; protected set; }
        public Vector3 LastMapUpdatePose { get; protected set; }
        public Vector3 LastScanMatchPose { get; protected set; }
        public Matrix4x4 LastScanMatchCov { get; protected set; }
        public float MinDistanceDiffForMapUpdate { get; set; }
        public float MinAngleDiffForMapUpdate { get; set; }

        /*
        DrawInterface* drawInterface;
        HectorDebugInfoInterface* debugInterface;
        */

        public float ScaleToMap => MapRep.ScaleToMap;
        public int MapLevels => MapRep.MapLevels;


        public GridMap GetGridMap(int mapLevel = 0)
        {
            return MapRep.GetGridMap(mapLevel);
        }

        public void AddMapMutex(int i, IMapLocker mapMutex)
        {
            MapRep.AddMapMutex(i, mapMutex);
        }

        public IMapLocker GetMapMutex(int i)
        {
            return MapRep.GetMapMutex(i);
        }

        public void SetUpdateFactorFree(float free_factor)
        {
            MapRep.SetUpdateFactorFree(free_factor);
        }

        public void SetUpdateFactorOccupied(float occupied_factor)
        {
            MapRep.SetUpdateFactorOccupied(occupied_factor);
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="mapResolution"></param>
        /// <param name="mapSizeX"></param>
        /// <param name="mapSizeY"></param>
        /// <param name="startCoords"></param>
        /// <param name="multiResSize"></param>
        /// <param name="drawInterface"></param>
        /// <param name="debugInterface"></param>
        public HectorSLAMProcessor(float mapResolution, int mapSizeX, int mapSizeY, Vector2 startCoords, int multiResSize, IDrawInterface drawInterface = null, IHectorDebugInfo debugInterface = null)
        {
            this.drawInterface = drawInterface;
            this.debugInterface = debugInterface;

            MapRep = new MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multiResSize, startCoords, drawInterface, debugInterface);

            Reset();

            MinDistanceDiffForMapUpdate = 0.4f * 1.0f;
            MinAngleDiffForMapUpdate = 0.13f * 1.0f;
        }

        /// <summary>
        /// Update with new data
        /// </summary>
        /// <param name="dataContainer">Scan data</param>
        /// <param name="poseHintWorld">Pose hint</param>
        /// <param name="mapWithoutMatching">Map without matching ?</param>
        public void Update(DataContainer dataContainer, Vector3 poseHintWorld, bool mapWithoutMatching = false)
        {
            System.Diagnostics.Debug.WriteLine($"ph: {poseHintWorld}");

            if (!mapWithoutMatching)
            {
                LastScanMatchPose = MapRep.MatchData(poseHintWorld, dataContainer, out Matrix4x4 lastScanMatchCov);
                LastScanMatchCov = lastScanMatchCov;
            }
            else
            {
                LastScanMatchPose = poseHintWorld;
            }

            //std::cout << "\nt1:\n" << newPoseEstimateWorld << "\n";
            //std::cout << "\n1";
            //std::cout << "\n" << lastScanMatchPose << "\n";

            if (Util.Util.PoseDifferenceLargerThan(LastScanMatchPose, LastMapUpdatePose, MinDistanceDiffForMapUpdate, MinAngleDiffForMapUpdate) || mapWithoutMatching)
            {
                MapRep.UpdateByScan(dataContainer, LastScanMatchPose);
                MapRep.OnMapUpdated();
                LastMapUpdatePose = LastScanMatchPose;
            }

            if (drawInterface != null)
            {
                GridMap gridMapRef = MapRep.GetGridMap(0);
                drawInterface.SetColor(1.0, 0.0, 0.0);
                drawInterface.SetScale(0.15);

                drawInterface.DrawPoint(gridMapRef.GetWorldCoords(Vector2.Zero));
                drawInterface.DrawPoint(gridMapRef.GetWorldCoords(gridMapRef.Dimensions.ToVector2()));
                drawInterface.DrawPoint(new Vector2(1.0f, 1.0f));

                drawInterface.SendAndResetData();
            }

            if (debugInterface != null)
            {
                debugInterface.SendAndResetData();
            }
        }

        /// <summary>
        /// Reset
        /// </summary>
        public void Reset()
        {
            LastMapUpdatePose = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            LastScanMatchPose = new Vector3(0.0f, 0.0f, 0.0f);

            MapRep.Reset();
        }
    }
}
