using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using BaseSLAM;
using HectorSLAM.Util;

namespace HectorSLAM.Map
{
    public class OccGridMapUtil
    {
        private readonly GridMapCacheArray cacheMethod;
        private readonly GridMap gridMap;
        private readonly List<Vector3> samplePoints; // TODO Unused, remove it
        private readonly CancellationTokenSource cts;
        private readonly HessianJob[] jobs;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="gridMap"></param>
        public OccGridMapUtil(GridMap gridMap)
        {
            cacheMethod = new GridMapCacheArray(gridMap.Dimensions);
            this.gridMap = gridMap;
            samplePoints = new List<Vector3>();

            cts = new CancellationTokenSource();

            // Create parallel threads to calculate hessian matrix derivatives
            jobs = new HessianJob[4];
            for (int i = 0; i < jobs.Length; i++)
            {
                jobs[i].Input = new SignalConcurrentQueue<HessianJobInput>();
                jobs[i].Output = new SignalConcurrentQueue<HessianJobOutput>();
                jobs[i].Thread = new Thread(new ParameterizedThreadStart(HessianJobLoop))
                {
                    Name = $"Hessian #{i + 1}"
                };
                jobs[i].Thread.Start(jobs[i]);
            }
        }

        public Vector3 GetWorldCoordsPose(Vector3 mapPose)
        {
            return gridMap.GetWorldCoordsPose(mapPose);
        }

        public Vector3 GetMapCoordsPose(Vector3 worldPose)
        {
            return gridMap.GetMapCoordsPose(worldPose);
        }

        public Vector2 GetWorldCoordsPoint(Vector2 mapPoint)
        {
            return gridMap.GetWorldCoords(mapPoint);
        }

        private struct HessianJob
        {
            public Thread Thread;
            public SignalConcurrentQueue<HessianJobInput> Input;
            public SignalConcurrentQueue<HessianJobOutput> Output;
        }

        public struct HessianJobInput
        {
            public Matrix3x2 Transformation;
            public float SinRotation;
            public float CosRotation;
            public IEnumerable<Vector2> Points;
        }

        private struct HessianJobOutput
        {
            public Matrix4x4 H;
            public Vector3 dTr;
        }

        /// <summary>
        /// Get complete hessian matrix derivatives
        /// </summary>
        /// <param name="pose"></param>
        /// <param name="scan"></param>
        /// <param name="H"></param>
        /// <param name="dTr"></param>
        public void GetCompleteHessianDerivs(Vector3 pose, ScanCloud scan, out Matrix4x4 H, out Vector3 dTr)
        {
            H = new Matrix4x4();
            dTr = Vector3.Zero;

            // Transformation of lidar measurements.
            // Translation is in pixels, need to convert it meters first.
            Matrix3x2 transform =
                Matrix3x2.CreateTranslation(pose.X * gridMap.Properties.CellLength, pose.Y * gridMap.Properties.CellLength) *
                Matrix3x2.CreateRotation(pose.Z) *
                Matrix3x2.CreateScale(gridMap.Properties.ScaleToMap);

            // Do the measurements scaling here, rather than wasting time in the rotDeriv calculation
            float sinRot = MathF.Sin(pose.Z) * gridMap.Properties.ScaleToMap;
            float cosRot = MathF.Cos(pose.Z) * gridMap.Properties.ScaleToMap;

            // Job chunk size (ceiling calculation)
            int chunkSize = (scan.Points.Count + jobs.Length - 1) / jobs.Length;
            
            // Feed input to hessian jobs
            for (int i = 0; i < jobs.Length; i++)
            {
                jobs[i].Input.Enqueue(new HessianJobInput()
                {
                    Transformation = transform,
                    SinRotation = sinRot,
                    CosRotation = cosRot,
                    Points = scan.Points.Skip(i * chunkSize).Take(chunkSize)
                });
            }

            // Wait for jobs completion (output data)
            WaitHandle.WaitAll(jobs.Select(j => j.Output.EnqueuedItemSignal).ToArray());

            // Aggragate jobs results
            for (int i = 0; i < jobs.Length; i++)
            {
                if (jobs[i].Output.TryDequeue(out HessianJobOutput output))
                {
                    H += output.H;
                    dTr += output.dTr;
                }
                else
                {
                    Debug.WriteLine($"Failed to get hessian job {i} output");
                }
            }

            // Symmetry for inversion
            H.M21 = H.M12;
            H.M31 = H.M13;
            H.M32 = H.M23;

            // Make 4x4 matrix inversible
            H.M44 = 1.0f;
        }

        /// <summary>
        /// Hessian job loop
        /// </summary>
        /// <param name="state">Job object</param>
        public void HessianJobLoop(object state)
        {
            HessianJob job = (HessianJob)state;
            WaitHandle[] waitHandles = new WaitHandle[] { cts.Token.WaitHandle, job.Input.EnqueuedItemSignal };

            while (!cts.IsCancellationRequested)
            {
                // Wait for job input
                switch (WaitHandle.WaitAny(waitHandles))
                {
                    // Cancellation ?
                    case 0:
                        break;

                    // Got input ?
                    case 1:
                        if (job.Input.TryDequeue(out HessianJobInput input))
                        {
                            PerformCompleteHessianDerivsJob(input, out HessianJobOutput output);
                            job.Output.Enqueue(output);
                        }
                        break;
                }
            }
        }

        /// <summary>
        /// Perform actual hessian derivatives job calculation
        /// </summary>
        /// <param name="input">Input data</param>
        /// <param name="output">Output data</param>
        private void PerformCompleteHessianDerivsJob(HessianJobInput input, out HessianJobOutput output)
        {
            output.dTr = Vector3.Zero;
            output.H = new Matrix4x4();

            foreach (Vector2 scanPoint in input.Points)
            {
                Vector2 scanPointMap = Vector2.Transform(scanPoint, input.Transformation);
                Vector3 transformedPointData = InterpMapValueWithDerivatives(scanPointMap);

                float funVal = 1.0f - transformedPointData.X;

                output.dTr.X += transformedPointData.Y * funVal;
                output.dTr.Y += transformedPointData.Z * funVal;

                float rotDeriv = ((-input.SinRotation * scanPoint.X - input.CosRotation * scanPoint.Y) * transformedPointData.Y +
                                   (input.CosRotation * scanPoint.X - input.SinRotation * scanPoint.Y) * transformedPointData.Z);

                output.dTr.Z += rotDeriv * funVal;

                output.H.M11 += transformedPointData.Y.Sqr();
                output.H.M22 += transformedPointData.Z.Sqr();
                output.H.M33 += rotDeriv.Sqr();

                output.H.M12 += transformedPointData.Y * transformedPointData.Z;
                output.H.M13 += transformedPointData.Y * rotDeriv;
                output.H.M23 += transformedPointData.Z * rotDeriv;
            }
        }

        /// <summary>
        /// ...
        /// </summary>
        /// <param name="coords">Map coordinates</param>
        /// <returns></returns>
        public Vector3 InterpMapValueWithDerivatives(Vector2 coords)
        {
            float[] intensities = new float[4];

            // Check if coords are within map limits.
            if (gridMap.Properties.IsPointOutOfMapBounds(coords))
            {
                return Vector3.Zero;
            }

            // Map coords are always positive, floor them by casting to int
            Point indMin = coords.ToFloorPoint();

            // get factors for bilinear interpolation
            Vector2 factors = coords - indMin.ToVector2();

            int sizeX = gridMap.Dimensions.X;
            int index = indMin.Y * sizeX + indMin.X;

            // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
            // filter gridPoint with gaussian and store in cache.
            if (!cacheMethod.ContainsCachedData(index, out intensities[0]))
            {
                intensities[0] = gridMap.GetGridProbabilityMap(index);
                cacheMethod.CacheData(index, intensities[0]);
            }

            ++index;

            if (!cacheMethod.ContainsCachedData(index, out intensities[1]))
            {
                intensities[1] = gridMap.GetGridProbabilityMap(index);
                cacheMethod.CacheData(index, intensities[1]);
            }

            index += sizeX - 1;

            if (!cacheMethod.ContainsCachedData(index, out intensities[2]))
            {
                intensities[2] = gridMap.GetGridProbabilityMap(index);
                cacheMethod.CacheData(index, intensities[2]);
            }

            ++index;

            if (!cacheMethod.ContainsCachedData(index, out intensities[3]))
            {
                intensities[3] = gridMap.GetGridProbabilityMap(index);
                cacheMethod.CacheData(index, intensities[3]);
            }

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

        public Matrix4x4 GetTransformForState(Vector3 transVector)
        {


            // return Eigen::Translation2f(transVector[0], transVector[1]) * Eigen::Rotation2Df(transVector[2]);
            return
                Matrix4x4.CreateTranslation(transVector.X, transVector.Y, 0) *
                Matrix4x4.CreateRotationZ(transVector.Z);
                //Matrix4x4.CreateScale(gridMap.Properties.ScaleToMap);
        }

        public void ResetCachedData()
        {
            cacheMethod.ResetCache();
        }

        public void ResetSamplePoints()
        {
            samplePoints.Clear();
        }
    }
}
