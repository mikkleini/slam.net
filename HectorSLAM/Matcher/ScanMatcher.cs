using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Text;
using Microsoft.Extensions.Logging;
using BaseSLAM;
using HectorSLAM.Main;
using HectorSLAM.Map;

namespace HectorSLAM.Matcher
{
    /// <summary>
    /// HectorSLAM scan matcher
    /// </summary>
    public class ScanMatcher : IDisposable
    {
        private readonly ILogger logger;
        private readonly ParallelWorker worker;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="numThreads">Number of calculation threads to use</param>
        /// <param name="logger">Logging interface</param>
        public ScanMatcher(int numThreads, ILogger logger = null)
        {
            this.logger = logger;
            worker = new ParallelWorker(numThreads, "HectorSLAM estimator");
        }

        /// <summary>
        /// Match scan data and get best position estimate.
        /// </summary>
        /// <param name="multiMap">Multi-level map</param>
        /// <param name="scan">Scanned points</param>
        /// <param name="hintPose">Estimation hint pose in world coordinates (meters)</param>
        /// <returns>Best position estimate in world coordinates (meters)</returns>
        public Vector3 MatchData(MapRepMultiMap multiMap, ScanCloud scan, Vector3 hintPose)
        {
            Vector3 estimate = hintPose;
            logger?.LogInformation($"Hint pose {hintPose.ToPoseString()}");

            // Start matching from coarsest map
            for (int index = multiMap.Maps.Length - 1; index >= 0; index--)
            {
                estimate = MatchData(multiMap.Maps[index], scan, estimate);
                logger?.LogInformation($"Layer {index} estimate: {estimate.ToPoseString()}");
            }

            return estimate;
        }

        /// <summary>
        /// Match scan data
        /// </summary>
        /// <param name="gridMap">Map</param>
        /// <param name="scan">Scanned points</param>
        /// <param name="hintPose">Estimation hint pose in world coordinates (meters)</param>
        /// <param name="numIterations">Number of estimate iterations to do</param>
        /// <returns>Best position estimate in world coordinates (meters)</returns>
        public Vector3 MatchData(OccGridMap gridMap, ScanCloud scan, Vector3 hintPose)
        {
            if (scan.Points.Count > 0)
            {
                Vector3 estimate = gridMap.GetMapCoordsPose(hintPose);

                for (int i = 0; i < gridMap.EstimateIterations; i++)
                {
                    EstimateTransformationLogLh(gridMap, scan, ref estimate);
                }

                // Normalize Z rotation
                estimate.Z = MathEx.NormalizeAngle(estimate.Z);

                // Return world coordinates
                return gridMap.GetWorldCoordsPose(estimate);
            }

            // When no scan points provided, return hint pose
            return hintPose;
        }

        /// <summary>
        /// Estimate pose
        /// </summary>
        /// <param name="gridMap">Map</param>
        /// <param name="scan">Scanned points</param>
        /// <param name="estimate">Estimate position</param>
        /// <returns>true if estimation was madde, false if failed</returns>
        protected bool EstimateTransformationLogLh(OccGridMap gridMap, ScanCloud scan, ref Vector3 estimate)
        {
            GetCompleteHessianDerivs(gridMap, scan, estimate, out Matrix4x4 H, out Vector3 dTr);

            if ((H.M11 != 0.0f) && (H.M22 != 0.0f))
            {
                if (!Matrix4x4.Invert(H, out Matrix4x4 iH))
                {
                    logger?.LogError($"Failed to calculate inverse matrix from {H}");
                    return false;
                }

                Vector3 searchDir = Vector3.Transform(dTr, iH);

                if (searchDir.Z > 0.2f)
                {
                    logger?.LogWarning($"SearchDir angle change ({MathEx.RadToDeg(searchDir.Z)}°) too large");
                    searchDir.Z = 0.2f;
                    
                }
                else if (searchDir.Z < -0.2f)
                {
                    logger?.LogWarning($"SearchDir angle change ({MathEx.RadToDeg(searchDir.Z)}°) too large");
                    searchDir.Z = -0.2f;
                }

                estimate += searchDir;

                return true;
            }

            return false;
        }

        /// <summary>
        /// Get complete hessian matrix derivatives
        /// </summary>
        /// <param name="gridMap">Map</param>
        /// <param name="scan">Scanned points</param>
        /// <param name="pose">Pose at which to calculate</param>
        /// <param name="H"></param>
        /// <param name="dTr"></param>
        protected void GetCompleteHessianDerivs(OccGridMap gridMap, ScanCloud scan, Vector3 pose, out Matrix4x4 H, out Vector3 dTr)
        {
            // Transformation of lidar measurements.
            // Translation is in pixels, need to convert it meters first.
            Matrix3x2 transform =
                Matrix3x2.CreateRotation(pose.Z) *
                Matrix3x2.CreateTranslation(pose.X * gridMap.Properties.CellLength, pose.Y * gridMap.Properties.CellLength) *
                Matrix3x2.CreateScale(gridMap.Properties.ScaleToMap);

            // Do the measurements scaling here, rather than wasting time in the rotDeriv calculation
            float sinRot = MathF.Sin(pose.Z) * gridMap.Properties.ScaleToMap;
            float cosRot = MathF.Cos(pose.Z) * gridMap.Properties.ScaleToMap;

            // Divide work to chunks
            int chunkSize = (scan.Points.Count + worker.NumThreads - 1) / worker.NumThreads;
            Vector3[] vectors = new Vector3[worker.NumThreads];
            Matrix4x4[] matrixes = new Matrix4x4[worker.NumThreads];

            // Do work in parallel threads
            worker.Work(index =>
            {
                var localH = new Matrix4x4();
                var localTr = Vector3.Zero;

                foreach (Vector2 scanPoint in scan.Points.Skip(index * chunkSize).Take(chunkSize))
                {
                    Vector2 scanPointMap = Vector2.Transform(scanPoint, transform);
                    Vector3 transformedPointData = InterpMapValueWithDerivatives(gridMap, scanPointMap);

                    float funVal = 1.0f - transformedPointData.X;

                    localTr.X += transformedPointData.Y * funVal;
                    localTr.Y += transformedPointData.Z * funVal;

                    float rotDeriv = ((-sinRot * scanPoint.X - cosRot * scanPoint.Y) * transformedPointData.Y +
                                       (cosRot * scanPoint.X - sinRot * scanPoint.Y) * transformedPointData.Z);

                    localTr.Z += rotDeriv * funVal;

                    localH.M11 += transformedPointData.Y.Sqr();
                    localH.M22 += transformedPointData.Z.Sqr();
                    localH.M33 += rotDeriv.Sqr();

                    localH.M12 += transformedPointData.Y * transformedPointData.Z;
                    localH.M13 += transformedPointData.Y * rotDeriv;
                    localH.M23 += transformedPointData.Z * rotDeriv;
                }

                matrixes[index] = localH;
                vectors[index] = localTr;
            });

            // Aggragate threaded results
            H = new Matrix4x4();
            dTr = Vector3.Zero;

            for (int i = 0; i < worker.NumThreads; i++)
            {
                H += matrixes[i];
                dTr += vectors[i];
            }

            // Symmetry for inversion
            H.M21 = H.M12;
            H.M31 = H.M13;
            H.M32 = H.M23;

            // Make 4x4 matrix inversible
            H.M44 = 1.0f;
        }

        /// <summary>
        /// ...
        /// </summary>
        /// <param name="coords">Map coordinates</param>
        /// <returns></returns>
        protected static Vector3 InterpMapValueWithDerivatives(OccGridMap gridMap, Vector2 coords)
        {
            float[] intensities = new float[4];

            // Check if coords are within map limits.
            if (gridMap.Properties.IsPointOutOfMapBounds(coords))
            {
                return Vector3.Zero;
            }

            // Map coords are always positive, floor them by casting to int
            Point indMin = coords.ToFloorPoint();

            // Get factors for bilinear interpolation
            Vector2 factors = coords - indMin.ToVector2();
            int sizeX = gridMap.Dimensions.X;
            int index = indMin.Y * sizeX + indMin.X;

            // Get grid values for the 4 grid points surrounding the current coords.
            intensities[0] = gridMap.GetCachedProbability(index);
            intensities[1] = gridMap.GetCachedProbability(index + 1);
            intensities[2] = gridMap.GetCachedProbability(index + sizeX);
            intensities[3] = gridMap.GetCachedProbability(index + sizeX + 1);

            float dx1 = intensities[0] - intensities[1];
            float dx2 = intensities[2] - intensities[3];

            float dy1 = intensities[0] - intensities[2];
            float dy2 = intensities[1] - intensities[3];

            float xFacInv = 1.0f - factors.X;
            float yFacInv = 1.0f - factors.Y;

            return new Vector3(
                ((intensities[0] * xFacInv + intensities[1] * factors.X) * yFacInv) +
                ((intensities[2] * xFacInv + intensities[3] * factors.X) * factors.Y),
                -((dx1 * xFacInv) + (dx2 * factors.X)),
                -((dy1 * yFacInv) + (dy2 * factors.Y)));
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
                worker.Dispose();
            }
        }
    }
}
