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

        public ScanMatcher(IDrawInterface drawInterface = null, IHectorDebugInfo debugInterface = null)
        {
            this.drawInterface = drawInterface;
            this.debugInterface = debugInterface;
        }

        public Vector3 MatchData(Vector3 beginEstimateWorld, OccGridMapUtil gridMapUtil, DataContainer dataContainer, out Matrix4x4 covMatrix, int maxIterations)
        {
            covMatrix = Matrix4x4.Identity;

            /*if (drawInterface != null)
            {
                drawInterface.SetScale(0.05f);
                drawInterface.SetColor(0.0f, 1.0f, 0.0f);
                drawInterface.DrawArrow(beginEstimateWorld);

                Vector3 beginEstimateMap = gridMapUtil.GetMapCoordsPose(beginEstimateWorld);

                DrawScan(beginEstimateMap, gridMapUtil, dataContainer);

                drawInterface.SetColor(1.0, 0.0, 0.0);
            }*/

            if (dataContainer.Count != 0)
            {
                Vector3 beginEstimateMap = gridMapUtil.GetMapCoordsPose(beginEstimateWorld);
                Vector3 estimate = beginEstimateMap;

                EstimateTransformationLogLh(ref estimate, gridMapUtil, dataContainer);

                for (int i = 0; i < maxIterations; ++i)
                {
                    EstimateTransformationLogLh(ref estimate, gridMapUtil, dataContainer);

                    /*
                    if (drawInterface != null)
                    {
                        float invNumIterf = 1.0f / maxIterations;
                        drawInterface.SetColor(i * invNumIterf, 0.0f, 0.0f);
                        drawInterface.DrawArrow(gridMapUtil.GetWorldCoordsPose(estimate));
                    }

                    if (debugInterface != null)
                    {
                        debugInterface.AddHessianMatrix(H);
                    }*/
                }

                /*
                if (drawInterface != null)
                {
                    drawInterface.SetColor(0.0, 0.0, 1.0);
                    DrawScan(estimate, gridMapUtil, dataContainer);
                }*/

                estimate.Z = Util.Util.NormalizeAngle(estimate.Z);
                covMatrix = H;

                return gridMapUtil.GetWorldCoordsPose(estimate);
            }

            return beginEstimateWorld;
        }

        protected bool EstimateTransformationLogLh(ref Vector3 estimate, OccGridMapUtil gridMapUtil, DataContainer dataPoints)
        {
            gridMapUtil.GetCompleteHessianDerivs(estimate, dataPoints, out Matrix4x4 H, out Vector3 dTr);

            if ((H.M11 != 0.0f) && (H.M22 != 0.0f))
            {
                if (!Matrix4x4.Invert(H, out Matrix4x4 iH))
                {
                    return false;
                }

                Vector3 searchDir = Vector3.Transform(dTr, iH);

                if (searchDir.Z > 0.2f)
                {
                    searchDir.Z = 0.2f;
                    System.Diagnostics.Debug.WriteLine("SearchDir angle change too large");
                }
                else if (searchDir.Z < -0.2f)
                {
                    searchDir.Z = -0.2f;
                    System.Diagnostics.Debug.WriteLine("SearchDir angle change too large");
                }

                //UpdateEstimatedPose(ref estimate, searchDir);
                estimate += searchDir;

                return true;
            }

            return false;
        }

        protected static void UpdateEstimatedPose(ref Vector3 estimate, Vector3 change)
        {
            estimate += change;
        }

        protected void DrawScan(Vector3 pose, OccGridMapUtil gridMapUtil, DataContainer dataContainer)
        {
            drawInterface.SetScale(0.02);
            Matrix3x2 transform = gridMapUtil.GetTransformForState(pose);

            for (int i = 0; i < dataContainer.Count; ++i)
            {
                drawInterface.DrawPoint(gridMapUtil.GetWorldCoordsPoint(Vector2.Transform(dataContainer[i], transform)));
            }
        }
    }
}
