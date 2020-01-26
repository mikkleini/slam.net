using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    public class OccGridMapUtil<ConcreteOccGridMap, ConcreteCacheMethod>
    {
        protected Vector4 intensities;
        protected ConcreteCacheMethod cacheMethod;
        protected ConcreteOccGridMap concreteGridMap;
        protected List<Vector3> samplePoints;
        protected int size = 0;
        protected float mapObstacleThreshold;

        public OccGridMapUtil(ConcreteOccGridMap gridMap)
        {
            concreteGridMap = gridMap;

            mapObstacleThreshold = gridMap->getObstacleThreshold();
            cacheMethod.setMapSize(gridMap->getMapDimensions());
        }

        public Vector3 GetWorldCoordsPose(Vector3 mapPose)
        {
            return concreteGridMap->getWorldCoordsPose(mapPose);
        }

        public Vector3 GetMapCoordsPose(Vector3 worldPose)
        {
            return concreteGridMap->getMapCoordsPose(worldPose);
        }

        public Vector2 GetWorldCoordsPoint(Vector2 mapPoint)
        {
            return concreteGridMap->getWorldCoords(mapPoint);
        }

        public void GetCompleteHessianDerivs(Vector3 pose, DataContainer dataPoints, Matrix4x4 H, Vector3 dTr)
        {
            int size = dataPoints.Count;

            Eigen::Affine2f transform(getTransformForState(pose));

            float sinRot = sin(pose[2]);
            float cosRot = cos(pose[2]);

            H = Eigen::Matrix3f::Zero();
            dTr = Vector3::Zero();

            for (int i = 0; i < size; ++i)
            {

                const Vector2& currPoint(dataPoints.getVecEntry(i));

                Vector3 transformedPointData(interpMapValueWithDerivatives(transform* currPoint));

                float funVal = 1.0f - transformedPointData[0];

                dTr[0] += transformedPointData[1] * funVal;
                dTr[1] += transformedPointData[2] * funVal;

                float rotDeriv = ((-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] + (cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);

                dTr[2] += rotDeriv * funVal;

                H(0, 0) += util::sqr(transformedPointData[1]);
                H(1, 1) += util::sqr(transformedPointData[2]);
                H(2, 2) += util::sqr(rotDeriv);

                H(0, 1) += transformedPointData[1] * transformedPointData[2];
                H(0, 2) += transformedPointData[1] * rotDeriv;
                H(1, 2) += transformedPointData[2] * rotDeriv;
            }

            H(1, 0) = H(0, 1);
            H(2, 0) = H(0, 2);
            H(2, 1) = H(1, 2);
        }

        public Matrix4x4 GetCovarianceForPose(Vector3 mapPose, DataContainer dataPoints)
        {
            float deltaTransX = 1.5f;
            float deltaTransY = 1.5f;
            float deltaAng = 0.05f;

            float x = mapPose.X;
            float y = mapPose.Y;
            float ang = mapPose.Z;

            Eigen::Matrix < float, 3, 7 > sigmaPoints;

            sigmaPoints.block < 3, 1 > (0, 0) = Vector3(x + deltaTransX, y, ang);
            sigmaPoints.block < 3, 1 > (0, 1) = Vector3(x - deltaTransX, y, ang);
            sigmaPoints.block < 3, 1 > (0, 2) = Vector3(x, y + deltaTransY, ang);
            sigmaPoints.block < 3, 1 > (0, 3) = Vector3(x, y - deltaTransY, ang);
            sigmaPoints.block < 3, 1 > (0, 4) = Vector3(x, y, ang + deltaAng);
            sigmaPoints.block < 3, 1 > (0, 5) = Vector3(x, y, ang - deltaAng);
            sigmaPoints.block < 3, 1 > (0, 6) = mapPose;

            Eigen::Matrix < float, 7, 1 > likelihoods;

            likelihoods[0] = getLikelihoodForState(Vector3(x + deltaTransX, y, ang), dataPoints);
            likelihoods[1] = getLikelihoodForState(Vector3(x - deltaTransX, y, ang), dataPoints);
            likelihoods[2] = getLikelihoodForState(Vector3(x, y + deltaTransY, ang), dataPoints);
            likelihoods[3] = getLikelihoodForState(Vector3(x, y - deltaTransY, ang), dataPoints);
            likelihoods[4] = getLikelihoodForState(Vector3(x, y, ang + deltaAng), dataPoints);
            likelihoods[5] = getLikelihoodForState(Vector3(x, y, ang - deltaAng), dataPoints);
            likelihoods[6] = getLikelihoodForState(Vector3(x, y, ang), dataPoints);

            float invLhNormalizer = 1 / likelihoods.sum();

            std::cout << "\n lhs:\n" << likelihoods;

            Vector3 mean(Vector3::Zero());

            for (int i = 0; i < 7; ++i)
            {
                mean += (sigmaPoints.block < 3, 1 > (0, i) * likelihoods[i]);
            }

            mean *= invLhNormalizer;

            Eigen::Matrix3f covMatrixMap(Eigen::Matrix3f::Zero());

            for (int i = 0; i < 7; ++i)
            {
                    Vector3 sigPointMinusMean(sigmaPoints.block<3, 1>(0, i) -mean);
                covMatrixMap += (likelihoods[i] * invLhNormalizer) * (sigPointMinusMean * (sigPointMinusMean.transpose()));
            }

            return covMatrixMap;

            //covMatrix.cwise() * invLhNormalizer;
            //transform = getTransformForState(Vector3(x-deltaTrans, y, ang);
        }

        public Matrix4x4 getCovMatrixWorldCoords(Matrix4x4 covMatMap)
        {
            //std::cout << "\nCovMap:\n" << covMatMap;

            Eigen::Matrix3f covMatWorld;

            float scaleTrans = concreteGridMap->getCellLength();
            float scaleTransSq = util::sqr(scaleTrans);

            covMatWorld(0, 0) = covMatMap(0, 0) * scaleTransSq;
            covMatWorld(1, 1) = covMatMap(1, 1) * scaleTransSq;

            covMatWorld(1, 0) = covMatMap(1, 0) * scaleTransSq;
            covMatWorld(0, 1) = covMatWorld(1, 0);

            covMatWorld(2, 0) = covMatMap(2, 0) * scaleTrans;
            covMatWorld(0, 2) = covMatWorld(2, 0);

            covMatWorld(2, 1) = covMatMap(2, 1) * scaleTrans;
            covMatWorld(1, 2) = covMatWorld(2, 1);

            covMatWorld(2, 2) = covMatMap(2, 2);

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

        public float getResidualForState(Vector3 state, DataContainer dataPoints)
        {
            int size = dataPoints.Count;
            int stepSize = 1;
            float residual = 0.0f;

            Eigen::Affine2f transform(getTransformForState(state));

            for (int i = 0; i < size; i += stepSize)
            {
                float funval = 1.0f - interpMapValue(transform * dataPoints.getVecEntry(i));
                residual += funval;
            }

            return residual;
        }

        public float getUnfilteredGridPoint(Point gridCoords)
        {
            return (concreteGridMap->getGridProbabilityMap(gridCoords.X, gridCoords.Y));
        }

        public float getUnfilteredGridPoint(int index)
        {
            return (concreteGridMap->getGridProbabilityMap(index));
        }

        public float interpMapValue(Vector2 coords)
        {
            //check if coords are within map limits.
            if (concreteGridMap->pointOutOfMapBounds(coords))
            {
                return 0.0f;
            }

            //map coords are alway positive, floor them by casting to int
            Point indMin(coords.cast<int>());

            //get factors for bilinear interpolation
            Vector2 factors(coords -indMin.cast<float>());

            int sizeX = concreteGridMap->getSizeX();

            int index = indMin[1] * sizeX + indMin[0];

            // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
            // filter gridPoint with gaussian and store in cache.
            if (!cacheMethod.containsCachedData(index, intensities[0]))
            {
                intensities[0] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[0]);
            }

            ++index;

            if (!cacheMethod.containsCachedData(index, intensities[1]))
            {
                intensities[1] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[1]);
            }

            index += sizeX - 1;

            if (!cacheMethod.containsCachedData(index, intensities[2]))
            {
                intensities[2] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[2]);
            }

            ++index;

            if (!cacheMethod.containsCachedData(index, intensities[3]))
            {
                intensities[3] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[3]);
            }

            float xFacInv = (1.0f - factors[0]);
            float yFacInv = (1.0f - factors[1]);

            return
              ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
              ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1]));

        }

        Vector3 interpMapValueWithDerivatives(const Vector2& coords)
        {
            //check if coords are within map limits.
            if (concreteGridMap->pointOutOfMapBounds(coords))
            {
                return Vector3(0.0f, 0.0f, 0.0f);
            }

            //map coords are always positive, floor them by casting to int
            Point indMin(coords.cast<int>());

            //get factors for bilinear interpolation
            Vector2 factors(coords -indMin.cast<float>());

            int sizeX = concreteGridMap->getSizeX();

            int index = indMin[1] * sizeX + indMin[0];

            // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
            // filter gridPoint with gaussian and store in cache.
            if (!cacheMethod.containsCachedData(index, intensities[0]))
            {
                intensities[0] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[0]);
            }

            ++index;

            if (!cacheMethod.containsCachedData(index, intensities[1]))
            {
                intensities[1] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[1]);
            }

            index += sizeX - 1;

            if (!cacheMethod.containsCachedData(index, intensities[2]))
            {
                intensities[2] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[2]);
            }

            ++index;

            if (!cacheMethod.containsCachedData(index, intensities[3]))
            {
                intensities[3] = getUnfilteredGridPoint(index);
                cacheMethod.cacheData(index, intensities[3]);
            }

            float dx1 = intensities[0] - intensities[1];
            float dx2 = intensities[2] - intensities[3];

            float dy1 = intensities[0] - intensities[2];
            float dy2 = intensities[1] - intensities[3];

            float xFacInv = (1.0f - factors[0]);
            float yFacInv = (1.0f - factors[1]);

            return Vector3(
              ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
              ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
              -((dx1 * xFacInv) + (dx2 * factors[0])),
              -((dy1 * yFacInv) + (dy2 * factors[1]))
            );
        }

        Eigen::Affine2f getTransformForState(const Vector3& transVector) const
          {
            return Eigen::Translation2f(transVector[0], transVector[1]) * Eigen::Rotation2Df(transVector[2]);
          }

          Eigen::Translation2f getTranslationForState(const Vector3& transVector) const
          {
            return Eigen::Translation2f(transVector[0], transVector[1]);
          }

        void resetCachedData()
        {
            cacheMethod.resetCache();
        }

        void resetSamplePoints()
        {
            samplePoints.clear();
        }
    }
}
