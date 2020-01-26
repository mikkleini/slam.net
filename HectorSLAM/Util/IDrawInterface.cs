using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Util
{
    public interface IDrawInterface
    {
        void DrawPoint(Vector2 pointWorldFrame);
        void DrawArrow(Vector3 poseWorld);
        void DrawCovariance(Vector2 mean, Matrix4x4 cov);
        void SetScale(double scale);
        void SetColor(double r, double g, double b, double a = 1.0);
        void SendAndResetData();
    }
}
