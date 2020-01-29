using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using HectorSLAM.Map;
using HectorSLAM.Scan;
using HectorSLAM.Util;

namespace HectorSLAM.Matcher
{
    public class ScanMatcher
    {
        private readonly IDrawInterface drawInterface;
        private readonly IHectorDebugInfo debugInterface;
        protected Vector3 dTr;
        protected Matrix4x4 H;

        //DrawInterface* drawInterface;
        //HectorDebugInfoInterface* debugInterface;

        public ScanMatcher(IDrawInterface drawInterface = null, IHectorDebugInfo debugInterface = null)
        {
            this.drawInterface = drawInterface;
            this.debugInterface = debugInterface;
        }

        public Vector3 MatchData(Vector3 beginEstimateWorld, OccGridMapUtil gridMapUtil, DataContainer dataContainer, out Matrix4x4 covMatrix, int maxIterations)
        {
            covMatrix = Matrix4x4.Identity;

            if (drawInterface != null)
            {
                drawInterface.SetScale(0.05f);
                drawInterface.SetColor(0.0f, 1.0f, 0.0f);
                drawInterface.DrawArrow(beginEstimateWorld);

                Vector3 beginEstimateMap = gridMapUtil.GetMapCoordsPose(beginEstimateWorld);

                DrawScan(beginEstimateMap, gridMapUtil, dataContainer);

                drawInterface.SetColor(1.0, 0.0, 0.0);
            }

            if (dataContainer.Count != 0)
            {
                Vector3 beginEstimateMap = gridMapUtil.GetMapCoordsPose(beginEstimateWorld);
                Vector3 estimate = beginEstimateMap;

                EstimateTransformationLogLh(ref estimate, gridMapUtil, dataContainer);
                //bool notConverged = estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

                /*
                const Eigen::Matrix2f& hessian (H.block<2,2>(0,0));


                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(hessian);

                const Vector2& eigValues (eig.eigenvalues());

                float cond = eigValues[1] / eigValues[0];
                float determinant = (hessian.determinant());
                */
                //std::cout << "\n cond: " << cond << " det: " << determinant << "\n";


                int numIter = maxIterations;

                for (int i = 0; i < numIter; ++i)
                {
                    //std::cout << "\nest:\n" << estimate;

                    EstimateTransformationLogLh(ref estimate, gridMapUtil, dataContainer);
                    //notConverged = estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

                    if (drawInterface != null)
                    {
                        float invNumIterf = 1.0f / numIter;
                        drawInterface.SetColor(i * invNumIterf, 0.0f, 0.0f);
                        drawInterface.DrawArrow(gridMapUtil.GetWorldCoordsPose(estimate));
                        //drawInterface->drawArrow(Vector3(0.0f, static_cast<float>(i)*0.05, 0.0f));
                    }

                    if (debugInterface != null)
                    {
                        debugInterface.AddHessianMatrix(H);
                    }
                }

                if (drawInterface != null)
                {
                    drawInterface.SetColor(0.0, 0.0, 1.0);
                    DrawScan(estimate, gridMapUtil, dataContainer);
                }


                /*
                Eigen::Matrix2f testMat(Eigen::Matrix2f::Identity());
                testMat(0,0) = 2.0f;

                float angleWorldCoords = util::toRad(30.0f);
                float sinAngle = sin(angleWorldCoords);
                float cosAngle = cos(angleWorldCoords);

                Eigen::Matrix2f rotMat;
                rotMat << cosAngle, -sinAngle, sinAngle, cosAngle;
                Eigen::Matrix2f covarianceRotated (rotMat * testMat * rotMat.transpose());

                drawInterface->setColor(0.0,0.0,1.0,0.5);
                drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(estimate.start<2>()), covarianceRotated);
                */



                /*
                Eigen::Matrix3f covMatMap (gridMapUtil.getCovarianceForPose(estimate, dataContainer));
                std::cout << "\nestim:" << estimate;
                std::cout << "\ncovMap\n" << covMatMap;
                drawInterface->setColor(0.0,0.0,1.0,0.5);


                Eigen::Matrix3f covMatWorld(gridMapUtil.getCovMatrixWorldCoords(covMatMap));
                 std::cout << "\ncovWorld\n" << covMatWorld;

                drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(estimate.start<2>()), covMatMap.block<2,2>(0,0));

                drawInterface->setColor(1.0,0.0,0.0,0.5);
                drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(estimate.start<2>()), covMatWorld.block<2,2>(0,0));

                std::cout << "\nH:\n" << H;

                float determinant = H.determinant();
                std::cout << "\nH_det: " << determinant;
                */

                /*
                Eigen::Matrix2f covFromHessian(H.block<2,2>(0,0) * 1.0f);
                //std::cout << "\nCovFromHess:\n" << covFromHessian;

                drawInterface->setColor(0.0, 1.0, 0.0, 0.5);
                drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(estimate.start<2>()),covFromHessian.inverse());

                Eigen::Matrix3f covFromHessian3d(H * 1.0f);
                //std::cout << "\nCovFromHess:\n" << covFromHessian;

                drawInterface->setColor(1.0, 0.0, 0.0, 0.8);
                drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(estimate.start<2>()),(covFromHessian3d.inverse()).block<2,2>(0,0));
                */


                estimate.Z = Util.Util.NormalizeAngle(estimate.Z);

                //covMatrix = Eigen::Matrix3f::Zero();
                //covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0).inverse());
                //covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0));


                /*
                covMatrix(0,0) = 1.0/(0.1*0.1);
                covMatrix(1,1) = 1.0/(0.1*0.1);
                covMatrix(2,2) = 1.0/((M_PI / 18.0f) * (M_PI / 18.0f));
                */

                covMatrix = H;

                return gridMapUtil.GetWorldCoordsPose(estimate);
            }

            return beginEstimateWorld;
        }

        protected bool EstimateTransformationLogLh(ref Vector3 estimate, OccGridMapUtil gridMapUtil, DataContainer dataPoints)
        {
            gridMapUtil.GetCompleteHessianDerivs(estimate, dataPoints, out Matrix4x4 H, out Vector3 dTr);
            //std::cout << "\nH\n" << H  << "\n";
            //std::cout << "\ndTr\n" << dTr  << "\n";

            if ((H.M11 != 0.0f) && (H.M22 != 0.0f))
            //if ((H(0, 0) != 0.0f) && (H(1, 1) != 0.0f))
            {
                //H += Eigen::Matrix3f::Identity() * 1.0f;
                if (!Matrix4x4.Invert(H, out Matrix4x4 iH))
                {
                    return false;
                }

                Vector3 searchDir = Vector3.Transform(dTr, iH);

                //std::cout << "\nsearchdir\n" << searchDir  << "\n";

                if (searchDir.Z > 0.2f)
                {
                    searchDir.Z = 0.2f;
                    Console.WriteLine("SearchDir angle change too large");
                }
                else if (searchDir.Z < -0.2f)
                {
                    searchDir.Z = -0.2f;
                    Console.WriteLine("SearchDir angle change too large");
                }

                UpdateEstimatedPose(ref estimate, searchDir);

                return true;
            }

            return false;
        }

        protected void UpdateEstimatedPose(ref Vector3 estimate, Vector3 change)
        {
            estimate += change;
        }

        protected void DrawScan(Vector3 pose, OccGridMapUtil gridMapUtil, DataContainer dataContainer)
        {
            drawInterface.SetScale(0.02);
            Matrix4x4 transform = gridMapUtil.GetTransformForState(pose);

            for (int i = 0; i < dataContainer.Count; ++i)
            {
                drawInterface.DrawPoint(gridMapUtil.GetWorldCoordsPoint(Vector2.Transform(dataContainer[i], transform)));
            }
        }
    }
}
