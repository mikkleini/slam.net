using BaseSLAM;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Util
{
    public static class Util
    {
        public static float NormalizeAnglePos(float angle)
        {
            float pi2 = MathF.PI * 2.0f;

            return ((angle % pi2) + pi2) % pi2;
        }

        public static float NormalizeAngle(float angle)
        {
            float a = NormalizeAnglePos(angle);

            if (a > MathF.PI)
            {
                a -= 2.0f * MathF.PI;
            }

            return a;
        }

        public static bool PoseDifferenceLargerThan(Vector3 pose1, Vector3 pose2, float distanceDiffThresh, float angleDiffThresh)
        {
            // Check distance
            if (Vector2.Distance(pose1.ToVector2(), pose2.ToVector2()) > distanceDiffThresh)
            {
                return true;
            }

            float angleDiff = pose1.Z - pose2.Z;

            if (angleDiff > MathF.PI)
            {
                angleDiff -= MathF.PI * 2.0f;
            }
            else if (angleDiff < -MathF.PI)
            {
                angleDiff += MathF.PI * 2.0f;
            }

            if (MathF.Abs(angleDiff) > angleDiffThresh)
            {
                return true;
            }

            return false;
        }
    }
}
