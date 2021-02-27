using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BaseSLAM
{
    /// <summary>
    /// Mathematical extensions
    /// </summary>
    public static class MathEx
    {
        /// <summary>
        /// Limit floating point number with minimum and maximum value
        /// </summary>
        /// <param name="x">Value</param>
        /// <param name="min">Minimum</param>
        /// <param name="max">Maximum</param>
        /// <returns>Limited value</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Limit(float x, float min, float max)
        {
            return MathF.Max(min, MathF.Min(max, x));
        }

        /// <summary>
        /// Limit integer number with minimum and maximum value
        /// </summary>
        /// <param name="x">Value</param>
        /// <param name="min">Minimum</param>
        /// <param name="max">Maximum</param>
        /// <returns>Limited value</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Limit(int x, int min, int max)
        {
            return Math.Max(min, Math.Min(max, x));
        }

        /// <summary>
        /// Convert degrees to radians
        /// </summary>
        /// <param name="deg">Degrees</param>
        /// <returns>Radians</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DegToRad(float deg)
        {
            return (deg * MathF.PI) / 180.0f;
        }

        /// <summary>
        /// Convert radians to degrees
        /// </summary>
        /// <param name="rad">Radians</param>
        /// <returns>Degrees</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float RadToDeg(float rad)
        {
            return (rad * 180.0f) / MathF.PI;
        }

        /// <summary>
        /// Signed difference between two angles in degrees
        /// From comments here:
        /// http://blog.lexique-du-net.com/index.php?post/Calculate-the-real-difference-between-two-angles-keeping-the-sign
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float DegDiff(float a, float b)
        {
            float d = ((a - b) + 180.0f) / 360.0f;
            return ((d - MathF.Floor(d)) * 360.0f) - 180.0f;
        }

        /// <summary>
        /// Signed difference between two angles in degrees
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static int DegDiff(int a, int b)
        {
            int d = ((a % 360) - (b % 360)) + 540;
            int r = (d / 360) * 360;
            return (d - r) - 180;
        }

        /// <summary>
        /// Signed difference between two angles in radians
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float RadDiff(float a, float b)
        {
            float d = ((a - b) + MathF.PI) / (2 * MathF.PI);
            return ((d - MathF.Floor(d)) * (2 * MathF.PI)) - MathF.PI;
        }

        /// <summary>
        /// Value in square
        /// </summary>
        /// <param name="val"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Sqr(this float val)
        {
            return val * val;
        }

        /// <summary>
        /// Normalize angle in radians. Results in positive angle (between 0 and 2 * PI)
        /// </summary>
        /// <param name="angle">Input angle in radians</param>
        /// <returns>Normalized angle in positive radians (0 to 2 * PI)</returns>
        private static float NormalizeAnglePos(float angle)
        {
            float pi2 = MathF.PI * 2.0f;

            return ((angle % pi2) + pi2) % pi2;
        }

        /// <summary>
        /// Normalize angle in radians.
        /// </summary>
        /// <param name="angle">Input angle in radians</param>
        /// <returns>Normalized angle in between -PI to +PI</returns>
        public static float NormalizeAngle(float angle)
        {
            float a = NormalizeAnglePos(angle);

            if (a > MathF.PI)
            {
                a -= 2.0f * MathF.PI;
            }

            return a;
        }
    }
}
