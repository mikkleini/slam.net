using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;
using HectorSLAM.Util;

namespace HectorSLAM.Main
{
    public class HectorSLAMProcessor
    {
        private readonly IDrawInterface drawInterface;
        private readonly IHectorDebugInfo debugInterface;

        public MapRepMultiMap MapRep { get; protected set; }
        public Vector3 LastMapUpdatePose { get; set; }
        public Vector3 LastScanMatchPose { get; set; }
        public Matrix4x4 LastScanMatchCov { get; protected set; }
        public float MinDistanceDiffForMapUpdate { get; set; }
        public float MinAngleDiffForMapUpdate { get; set; }

  
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
        /// <param name="mapResolution">Meters per pixel</param>
        /// <param name="mapSizeX">Size X in pixels</param>
        /// <param name="mapSizeY">Size Y in pixels</param>
        /// <param name="startCoords">Start coordinates as fraction of whole map size</param>
        /// <param name="multiResSize"></param>
        /// <param name="drawInterface"></param>
        /// <param name="debugInterface"></param>
        public HectorSLAMProcessor(float mapResolution, int mapSizeX, int mapSizeY, Vector2 startCoords, int multiResSize, IDrawInterface drawInterface = null, IHectorDebugInfo debugInterface = null)
        {
            this.drawInterface = drawInterface;
            this.debugInterface = debugInterface;

            MapRep = new MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multiResSize, startCoords, drawInterface, debugInterface);

            Reset();

            MinDistanceDiffForMapUpdate = 0.3f * 1.0f;
            MinAngleDiffForMapUpdate = 0.13f * 1.0f;
        }

        /// <summary>
        /// Update with new data
        /// </summary>
        /// <param name="scan">Scanned cloud points</param>
        /// <param name="poseHintWorld">Pose hint</param>
        /// <param name="mapWithoutMatching">Map without matching ?</param>
        public void Update(ScanCloud scan, Vector3 poseHintWorld, bool mapWithoutMatching = false)
        {
            //System.Diagnostics.Debug.WriteLine($"ph: {poseHintWorld}");

            if (!mapWithoutMatching)
            {
                Stopwatch w = new Stopwatch();
                w.Start();                
                LastScanMatchPose = MapRep.MatchData(poseHintWorld, scan, out Matrix4x4 lastScanMatchCov);
                System.Diagnostics.Debug.WriteLine($"Match time: {w.ElapsedTicks}"); // 600-700, 1000

                LastScanMatchCov = lastScanMatchCov;
            }
            else
            {
                LastScanMatchPose = poseHintWorld;
            }

            // Update map(s) when:
            //    Position or rotation has changed significantly.
            //    Mapping is requested.
            if (Vector2.DistanceSquared(LastScanMatchPose.ToVector2(), LastMapUpdatePose.ToVector2()) > MinDistanceDiffForMapUpdate.Sqr() ||
                (MathEx.DegDiff(LastScanMatchPose.Z, LastMapUpdatePose.Z) > MinAngleDiffForMapUpdate) ||
                mapWithoutMatching)
            {
                MapRep.UpdateByScan(scan, LastScanMatchPose);
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
