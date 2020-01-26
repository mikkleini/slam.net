using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using HectorSLAM.Map;

namespace HectorSLAM.Matcher
{
    public class ScanMatcher<T>
    {
        protected Vector3 dTr;
        protected Matrix4x4 H;

        //DrawInterface* drawInterface;
        //HectorDebugInfoInterface* debugInterface;

        public ScanMatcher(DrawInterface* drawInterfaceIn = 0, HectorDebugInfoInterface* debugInterfaceIn = 0)
        {
        }

        public Vector3 MatchData(Vector3 beginEstimateWorld, ConcreteOccGridMapUtil gridMapUtil, DataContainer dataContainer, Matrix4x4 covMatrix, int maxIterations)
        {
            /*
            if (drawInterface)
            {
                drawInterface->setScale(0.05f);
                drawInterface->setColor(0.0f,1.0f, 0.0f);
                drawInterface->drawArrow(beginEstimateWorld);

                Vector3 beginEstimateMap(gridMapUtil.getMapCoordsPose(beginEstimateWorld));

                drawScan(beginEstimateMap, gridMapUtil, dataContainer);

                drawInterface->setColor(1.0,0.0,0.0);
            }*/

            if (dataContainer.Count != 0)
            {
                Vector3 beginEstimateMap = gridMapUtil.getMapCoordsPose(beginEstimateWorld);

                Vector3 estimate = beginEstimateMap;

                estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);
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

                for (int i = 0; i<numIter; ++i)
                {
                    //std::cout << "\nest:\n" << estimate;

                    estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);
                    //notConverged = estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

                    /*
                    if (drawInterface)
                    {
                        float invNumIterf = 1.0f / static_cast<float>(numIter);
                        drawInterface->setColor(static_cast<float>(i) * invNumIterf,0.0f, 0.0f);
                        drawInterface->drawArrow(gridMapUtil.getWorldCoordsPose(estimate));
                        //drawInterface->drawArrow(Vector3(0.0f, static_cast<float>(i)*0.05, 0.0f));
                    }

                    if(debugInterface)
                    {
                        debugInterface->addHessianMatrix(H);
                    }
                    */
                }

                /*
                if (drawInterface)
                {
                    drawInterface->setColor(0.0,0.0,1.0);
                    drawScan(estimate, gridMapUtil, dataContainer);
                }*/


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


                estimate[2] = util::normalize_angle(estimate[2]);

                covMatrix = Eigen::Matrix3f::Zero();
                //covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0).inverse());
                //covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0));


                /*
                covMatrix(0,0) = 1.0/(0.1*0.1);
                covMatrix(1,1) = 1.0/(0.1*0.1);
                covMatrix(2,2) = 1.0/((M_PI / 18.0f) * (M_PI / 18.0f));
                */

                covMatrix = H;

                return gridMapUtil.getWorldCoordsPose(estimate);
            }

            return beginEstimateWorld;
        }

        protected bool estimateTransformationLogLh(Vector3 estimate, ConcreteOccGridMapUtil gridMapUtil, DataContainer dataPoints)
        {
            gridMapUtil.getCompleteHessianDerivs(estimate, dataPoints, H, dTr);
            //std::cout << "\nH\n" << H  << "\n";
            //std::cout << "\ndTr\n" << dTr  << "\n";


            if ((H(0, 0) != 0.0f) && (H(1, 1) != 0.0f))
            {
                //H += Eigen::Matrix3f::Identity() * 1.0f;
                Vector3 searchDir = H.inverse() * dTr;

                //std::cout << "\nsearchdir\n" << searchDir  << "\n";

                if (searchDir[2] > 0.2f)
                {
                    searchDir[2] = 0.2f;
                    std::cout << "SearchDir angle change too large\n";
                }
                else if (searchDir[2] < -0.2f)
                {
                    searchDir[2] = -0.2f;
                    std::cout << "SearchDir angle change too large\n";
                }

                updateEstimatedPose(estimate, searchDir);
                return true;
            }
            return false;
        }

        protected void updateEstimatedPose(ref Vector3 estimate, Vector3 change)
        {
            estimate += change;
        }

        protected void drawScan(Vector3 pose, ConcreteOccGridMapUtil gridMapUtil, DataContainer dataContainer)
        {
            drawInterface->setScale(0.02);

            Eigen::Affine2f transform(gridMapUtil.getTransformForState(pose));

            int size = dataContainer.getSize();
            for (int i = 0; i < size; ++i)
            {
                const Vector2&currPoint(dataContainer.getVecEntry(i));
                drawInterface->drawPoint(gridMapUtil.getWorldCoordsPoint(transform * currPoint));
            }
        }
    }
}
