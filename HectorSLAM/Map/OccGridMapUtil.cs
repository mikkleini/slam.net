using HectorSLAM.Scan;
using HectorSLAM.Util;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    public class OccGridMapUtil
    {
        private readonly GridMapCacheArray cacheMethod;
        private readonly GridMap gridMap;
        private readonly List<Vector3> samplePoints;
        private readonly float[] intensities = new float[4] { 0, 0, 0, 0 };
        private readonly float mapObstacleThreshold;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="gridMap"></param>
        public OccGridMapUtil(GridMap gridMap)
        {
            cacheMethod = new GridMapCacheArray();
            this.gridMap = gridMap;
            samplePoints = new List<Vector3>();

            mapObstacleThreshold = gridMap.GetObstacleThreshold();
            cacheMethod.SetMapSize(gridMap.Dimensions);
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

        public void GetCompleteHessianDerivs(Vector3 pose, DataContainer dataPoints, out Matrix4x4 H, out Vector3 dTr)
        {
            Matrix4x4 transform = GetTransformForState(pose);

            float sinRot = MathF.Sin(pose.Z);
            float cosRot = MathF.Cos(pose.Z);

            H = new Matrix4x4();
            dTr = Vector3.Zero;

            foreach (Vector2 currPoint in dataPoints)
            {
                Vector3 transformedPointData = InterpMapValueWithDerivatives(Vector2.Transform(currPoint, transform));

                float funVal = 1.0f - transformedPointData.X;

                dTr.X += transformedPointData.Y * funVal;
                dTr.Y += transformedPointData.Z * funVal;

                float rotDeriv = ((-sinRot * currPoint.X - cosRot * currPoint.Y) * transformedPointData.Y +
                                   (cosRot * currPoint.X - sinRot * currPoint.Y) * transformedPointData.Z);

                dTr.Z += rotDeriv * funVal;

                H.M11 += Util.Util.Sqr(transformedPointData.Y);
                H.M22 += Util.Util.Sqr(transformedPointData.Z);
                H.M33 += Util.Util.Sqr(rotDeriv);

                H.M12 += transformedPointData.Y * transformedPointData.Z;
                H.M13 += transformedPointData.Y * rotDeriv;
                H.M23 += transformedPointData.Z * rotDeriv;
            }

            H.M21 = H.M12;
            H.M31 = H.M13;
            H.M32 = H.M23;
        }

        public Matrix4x4 GetCovarianceForPose(Vector3 mapPose, DataContainer dataPoints)
        {
            float deltaTransX = 1.5f;
            float deltaTransY = 1.5f;
            float deltaAng = 0.05f;

            float x = mapPose.X;
            float y = mapPose.Y;
            float ang = mapPose.Z;

            Vector3[] sigmaPoints = new Vector3[7]
            {
                new Vector3(x + deltaTransX, y, ang),
                new Vector3(x - deltaTransX, y, ang),
                new Vector3(x, y + deltaTransY, ang),
                new Vector3(x, y - deltaTransY, ang),
                new Vector3(x, y, ang + deltaAng),
                new Vector3(x, y, ang - deltaAng),
                mapPose
            };

            float[] likelihoods = new float[7]
            {
                GetLikelihoodForState(new Vector3(x + deltaTransX, y, ang), dataPoints),
                GetLikelihoodForState(new Vector3(x - deltaTransX, y, ang), dataPoints),
                GetLikelihoodForState(new Vector3(x, y + deltaTransY, ang), dataPoints),
                GetLikelihoodForState(new Vector3(x, y - deltaTransY, ang), dataPoints),
                GetLikelihoodForState(new Vector3(x, y, ang + deltaAng), dataPoints),
                GetLikelihoodForState(new Vector3(x, y, ang - deltaAng), dataPoints),
                GetLikelihoodForState(new Vector3(x, y, ang), dataPoints)
            };

            float invLhNormalizer = 1 / likelihoods.Sum();

            System.Diagnostics.Debug.WriteLine($"lhs: {likelihoods}");

            Vector3 mean = Vector3.Zero;

            for (int i = 0; i < 7; ++i)
            {
                mean += sigmaPoints[i] * likelihoods[i];
            }

            mean *= invLhNormalizer;

            Matrix4x4 covMatrixMap = new Matrix4x4();

            for (int i = 0; i < 7; ++i)
            {
                Vector3 sigPointMinusMean = sigmaPoints[i] - mean;
                Matrix4x4 sp = new Matrix4x4(sigPointMinusMean.X, sigPointMinusMean.Y, sigPointMinusMean.Z, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                Matrix4x4 spt = Matrix4x4.Transpose(sp);
                covMatrixMap += Matrix4x4.Multiply(Matrix4x4.Multiply(sp, spt), likelihoods[i] * invLhNormalizer);

                // Original loop:
                // Eigen::Vector3f sigPointMinusMean(sigmaPoints.block<3, 1>(0, i) -mean);
                // covMatrixMap += (likelihoods[i] * invLhNormalizer) * (sigPointMinusMean * (sigPointMinusMean.transpose()));
            }

            return covMatrixMap;
        }

        public Matrix4x4 GetCovMatrixWorldCoords(Matrix4x4 covMatMap)
        {
            //std::cout << "\nCovMap:\n" << covMatMap;

            Matrix4x4 covMatWorld = new Matrix4x4();

            float scaleTrans = gridMap.DimensionProperties.CellLength;
            float scaleTransSq = Util.Util.Sqr(scaleTrans);

            covMatWorld.M11 = covMatMap.M11 * scaleTransSq;
            covMatWorld.M22 = covMatMap.M22 * scaleTransSq;

            covMatWorld.M21 = covMatMap.M21 * scaleTransSq;
            covMatWorld.M12 = covMatWorld.M21;

            covMatWorld.M31 = covMatMap.M31 * scaleTrans;
            covMatWorld.M13 = covMatWorld.M31;

            covMatWorld.M32 = covMatMap.M32 * scaleTrans;
            covMatWorld.M23 = covMatWorld.M32;

            covMatWorld.M33 = covMatMap.M33;

            return covMatWorld;
        }

        public float GetLikelihoodForState(Vector3 state, DataContainer dataPoints)
        {
            float resid = GetResidualForState(state, dataPoints);

            return GetLikelihoodForResidual(resid, dataPoints.Count);
        }

        public float GetLikelihoodForResidual(float residual, int numDataPoints)
        {
            return 1.0f - (residual / (float)numDataPoints);
        }

        public float GetResidualForState(Vector3 state, DataContainer dataPoints)
        {
            int stepSize = 1;
            float residual = 0.0f;

            Matrix4x4 transform = GetTransformForState(state);

            for (int i = 0; i < dataPoints.Count; i += stepSize)
            {
                float funval = 1.0f - InterpMapValue(Vector2.Transform(dataPoints[i], transform));
                residual += funval;
            }

            return residual;
        }

        public float GetUnfilteredGridPoint(Point gridCoords)
        {
            return gridMap.GetGridProbabilityMap(gridCoords.X + gridCoords.Y * gridMap.Dimensions.X);
        }

        public float GetUnfilteredGridPoint(int index)
        {
            return gridMap.GetGridProbabilityMap(index);
        }

        public float InterpMapValue(Vector2 coords)
        {
            // Check if coords are within map limits.
            if (gridMap.DimensionProperties.IsPointOutOfMapBounds(coords))
            {
                return 0.0f;
            }

            // map coords are alway positive, floor them by casting to int
            Point indMin = coords.ToFloorPoint();

            // get factors for bilinear interpolation
            Vector2 factors = coords - indMin.ToVector2();

            int sizeX = gridMap.Dimensions.X;
            int index = indMin.Y * sizeX + indMin.X;

            // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
            // filter gridPoint with gaussian and store in cache.
            if (!cacheMethod.ContainsCachedData(index, out intensities[0]))
            {
                intensities[0] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[0]);
            }

            ++index;

            if (!cacheMethod.ContainsCachedData(index, out intensities[1]))
            {
                intensities[1] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[1]);
            }

            index += sizeX - 1;

            if (!cacheMethod.ContainsCachedData(index, out intensities[2]))
            {
                intensities[2] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[2]);
            }

            ++index;

            if (!cacheMethod.ContainsCachedData(index, out intensities[3]))
            {
                intensities[3] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[3]);
            }

            float xFacInv = (1.0f - factors.X);
            float yFacInv = (1.0f - factors.Y);

            return
                  ((intensities[0] * xFacInv + intensities[1] * factors.X) * (yFacInv)) +
                  ((intensities[1] * xFacInv + intensities[3] * factors.X) * (factors.Y));
        }

        public Vector3 InterpMapValueWithDerivatives(Vector2 coords)
        {
            // Check if coords are within map limits.
            if (gridMap.DimensionProperties.IsPointOutOfMapBounds(coords))
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
                intensities[0] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[0]);
            }

            ++index;

            if (!cacheMethod.ContainsCachedData(index, out intensities[1]))
            {
                intensities[1] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[1]);
            }

            index += sizeX - 1;

            if (!cacheMethod.ContainsCachedData(index, out intensities[2]))
            {
                intensities[2] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[2]);
            }

            ++index;

            if (!cacheMethod.ContainsCachedData(index, out intensities[3]))
            {
                intensities[3] = GetUnfilteredGridPoint(index);
                cacheMethod.CacheData(index, intensities[3]);
            }

            float dx1 = intensities[0] - intensities[1];
            float dx2 = intensities[2] - intensities[3];

            float dy1 = intensities[0] - intensities[2];
            float dy2 = intensities[1] - intensities[3];

            float xFacInv = (1.0f - factors.X);
            float yFacInv = (1.0f - factors.Y);

            return new Vector3(
                ((intensities[0] * xFacInv + intensities[1] * factors.X) * (yFacInv)) +
                ((intensities[2] * xFacInv + intensities[3] * factors.X) * (factors.Y)),
                -((dx1 * xFacInv) + (dx2 * factors.X)),
                -((dy1 * yFacInv) + (dy2 * factors.Y)));
        }

        public Matrix4x4 GetTransformForState(Vector3 transVector)
        {
            return Matrix4x4.CreateTranslation(transVector.X, transVector.Y, 0.0f) * Matrix4x4.CreateRotationZ(transVector.Z);
        }

        public Matrix4x4 getTranslationForState(Vector3 transVector)
        {
            return Matrix4x4.CreateTranslation(transVector.X, transVector.Y, 0.0f);
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
