using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Numerics;
using System.Text;
using Microsoft.Extensions.Logging;
using BaseSLAM;
using HectorSLAM.Map;
using HectorSLAM.Matcher;

namespace HectorSLAM.Main
{
    /// <summary>
    /// Hector SLAM processor
    /// </summary>
    public class HectorSLAMProcessor : IDisposable
    {
        private readonly ILogger logger;
        private readonly Vector3 startPose;
        private readonly ScanMatcher scanMatcher;

        /// <summary>
        /// Multi-level map
        /// </summary>
        public MapRepMultiMap MapRep { get; private set; }

        /// <summary>
        /// Last map updatep ose
        /// </summary>
        public Vector3 LastMapUpdatePose { get; private set; }

        /// <summary>
        /// Last scan match pose
        /// </summary>
        public Vector3 MatchPose { get; private set; }

        /// <summary>
        /// Average match timing in milliseconds
        /// </summary>
        public float MatchTiming { get; private set; }

        /// <summary>
        /// Average map update timing in milliseconds
        /// </summary>
        public float UpdateTiming { get; private set; }

        /// <summary>
        /// Distance difference (in meters) required for map update
        /// </summary>
        public float MinDistanceDiffForMapUpdate { get; set; } = 0.3f;

        /// <summary>
        /// Angular difference (in radians) required for map update
        /// </summary>
        public float MinAngleDiffForMapUpdate { get; set; } = 0.13f;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="mapResolution">Meters per pixel</param>
        /// <param name="mapSize">Map size in pixels</param>
        /// <param name="startPose">Start pose (X and Y in meters, Z in degrees)</param>
        /// <param name="numDepth">Number of maps</param>
        /// <param name="numThreads">Number of processing threads</param>
        public HectorSLAMProcessor(float mapResolution, Point mapSize, Vector3 startPose, int numDepth, int numThreads, ILogger logger = null)
        {
            this.logger = logger;
            this.startPose = startPose;

            MapRep = new MapRepMultiMap(mapResolution, mapSize, numDepth, Vector2.Zero);
            scanMatcher = new ScanMatcher(numThreads, logger);

            // Set initial poses
            MatchPose = startPose;
            LastMapUpdatePose = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        }

        /// <summary>
        /// Update map with new scan data and search for the best pose estimate.
        /// </summary>
        /// <param name="scan">Scanned cloud points</param>
        /// <param name="poseHintWorld">Pose hint</param>
        /// <param name="mapWithoutMatching">Map without matching ?</param>
        /// <returns>true if map was updated, false if not</returns>
        public bool Update(ScanCloud scan, Vector3 poseHintWorld, bool mapWithoutMatching = false)
        {
            // Do position matching or not ?
            if (!mapWithoutMatching)
            {
                // Match and measure the performance
                var watch = Stopwatch.StartNew();
                MatchPose = scanMatcher.MatchData(MapRep, scan, poseHintWorld);

                // Calculate average timing
                MatchTiming = (3.0f * MatchTiming + (float)watch.Elapsed.TotalMilliseconds) / 4.0f;
            }
            else
            {
                MatchPose = poseHintWorld;
            }

            // Update map(s) when:
            //    Map hasn't been updated yet
            //    Position or rotation has changed significantly.
            //    Mapping is requested.
            if (Vector2.DistanceSquared(MatchPose.ToVector2(), LastMapUpdatePose.ToVector2()) > MinDistanceDiffForMapUpdate.Sqr() ||
                (MathEx.DegDiff(MatchPose.Z, LastMapUpdatePose.Z) > MinAngleDiffForMapUpdate) ||
                mapWithoutMatching)
            {
                var watch = Stopwatch.StartNew();
                MapRep.UpdateByScan(scan, MatchPose);

                // Calculate average timing
                UpdateTiming = (3.0f * UpdateTiming + (float)watch.Elapsed.TotalMilliseconds) / 4.0f;

                // Remember update pose
                LastMapUpdatePose = MatchPose;

                // Notify about update
                logger?.LogInformation($"Map update at {MatchPose.ToPoseString()}");
                return true;
            }

            return false;
        }

        /// <summary>
        /// Reset
        /// </summary>
        public void Reset()
        {
            MapRep.Reset();

            // Set initial poses
            MatchPose = startPose;
            LastMapUpdatePose = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        }

        /// <summary>
        /// Dispose function
        /// </summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Internal disposing function.
        /// </summary>
        /// <param name="disposing"></param>
        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                scanMatcher.Dispose();
            }
        }
    }
}
