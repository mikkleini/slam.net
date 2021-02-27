using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Threading;
using System.Numerics;
using Redzen.Numerics.Distributions.Float;
using BaseSLAM;

namespace CoreSLAM
{
    /// <summary>
    /// Core SLAM processor
    /// </summary>
    public class CoreSLAMProcessor : IDisposable
    {
        // Constants
        private const ushort TS_NO_OBSTACLE = 65500;
        private const ushort TS_OBSTACLE = 0;

        // Private read only objects
        private readonly Vector3 startPose;
        private readonly bool[,] noHitMap;
        private readonly ZigguratGaussianSampler samplerXY;
        private readonly ZigguratGaussianSampler samplerTheta;
        private readonly ParallelWorker worker;
        private readonly Queue<float>[] randomQueuesXY;
        private readonly Queue<float>[] randomQueuesTheta;

        // Private variables
        private int scanCount;
        private Vector3 lastOdometryPose;

        /// <summary>
        /// Physical square map size (length of edge) in meters
        /// </summary>
        public float PhysicalMapSize { get; }

        /// <summary>
        /// Hole map
        /// </summary>
        public HoleMap HoleMap { get; }

        /// <summary>
        /// Obstacle map
        /// </summary>
        public ObstacleMap ObstacleMap { get; }


        /// <summary>
        /// Search range at X and Y coordinates in meters
        /// </summary>
        public float SigmaXY { get; }

        /// <summary>
        /// Search span at Theta orentation in radians
        /// </summary>
        public float SigmaTheta { get; }

        /// <summary>
        /// Number of search iterations per search thread.
        /// </summary>
        public int SearchIterationsPerThread { get; }

        /// <summary>
        /// Number of search threads
        /// </summary>
        public int NumSearchThreads { get; }

        /// <summary>
        /// Map quality (1 to 255)
        /// DO NOT USE 0 OR 256!
        /// This is actually a alpha factor in drawing which
        /// affects how fast the map get updated with new lidar rays.
        /// The bigger number, the faster.
        /// </summary>
        public byte Quality { get; set; } = 50;

        /// <summary>
        /// Hole width in meters
        /// </summary>
        public float HoleWidth { get; set; } = 0.6f;

        /// <summary>
        /// Threshold of scan count after which start searching position
        /// </summary>
        public int PositionSearchBeginning { get; set; } = 5;

        /// <summary>
        /// Number of hits or no-hits to reach per pixel to mark obstacle map pixel clear or occupied.
        /// After changing this, Reset function has to be called!
        /// </summary>
        public sbyte UnmappedObstacleHits { get; set; } = -5;

        /// <summary>
        /// Maximum number of obsacle hits in obstacle map to count
        /// </summary>
        public sbyte MaxObstacleHits { get; set; } = 10;

        /// <summary>
        /// Pose
        /// </summary>
        public Vector3 Pose { get; private set; } = Vector3.Zero;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="physicalMapSize">Physical map size in meters</param>
        /// <param name="holeMapSize">Hole map size in pixels</param>
        /// <param name="obstacleMapSize">Obstacle map size in pixels</param>
        /// <param name="startPose">Start pose</param>
        /// <param name="sigmaXY">Sigma XY in meters</param>
        /// <param name="sigmaTheta">Sigma theta in radians</param>
        /// <param name="iterationsPerThread">Search iterations per thread</param>
        /// <param name="numSearchThreads">Number of search threads (1 or less for no threading)</param>
        public CoreSLAMProcessor(float physicalMapSize, int holeMapSize, int obstacleMapSize, Vector3 startPose,
            float sigmaXY, float sigmaTheta, int iterationsPerThread, int numSearchThreads)
        {
            // Set properties
            PhysicalMapSize = physicalMapSize;
            this.startPose = startPose;
            SigmaXY = sigmaXY;
            SigmaTheta = sigmaTheta;
            SearchIterationsPerThread = iterationsPerThread;
            NumSearchThreads = numSearchThreads;

            // Create maps
            HoleMap = new HoleMap(holeMapSize, physicalMapSize);
            ObstacleMap = new ObstacleMap(obstacleMapSize, physicalMapSize);
            noHitMap = new bool[obstacleMapSize, obstacleMapSize];

            // Use 3rd party library for fast normal distribution random number generator
            samplerXY = new ZigguratGaussianSampler(0.0f, sigmaXY);
            samplerTheta = new ZigguratGaussianSampler(0.0f, sigmaTheta);

            // Reset everything
            Reset();

            // Threaded mode or not ?
            if (numSearchThreads <= 0)
            {
                worker = null;
            }
            else
            {
                worker = new ParallelWorker(NumSearchThreads, "CoreSLAM search");
                randomQueuesXY = new Queue<float>[NumSearchThreads];
                randomQueuesTheta = new Queue<float>[NumSearchThreads];

                // Create random number queues and fill them up
                for (int i = 0; i < NumSearchThreads; i++)
                {
                    randomQueuesXY[i] = new Queue<float>();
                    randomQueuesTheta[i] = new Queue<float>();

                    FillRandomQueues(i);
                }
            }
        }

        /// <summary>
        /// Reset everything
        /// </summary>
        public void Reset()
        {
            Array.Fill(HoleMap.Pixels, (ushort)((TS_OBSTACLE + TS_NO_OBSTACLE) / 2));
            ArrayEx.Fill(ObstacleMap.Pixels, UnmappedObstacleHits);

            Pose = startPose;
            lastOdometryPose = Vector3.Zero;
            scanCount = 0;
        }

        /// <summary>
        /// Create scan points cloud from scan segments
        /// Each segment is taken from different pose - it happens when robot is moving.
        /// That means there's no single center point.
        /// 
        /// Segments poses should be relative to the latest position of robot
        /// </summary>
        /// <param name="segments">Scan segments</param>
        /// <param name="odometryPose">Last odometry pose</param>
        /// <param name="cloud">Scan cloud</param>
        private static void ScanSegmentsToCloud(IEnumerable<ScanSegment> segments, Vector3 odometryPose, out ScanCloud cloud)
        {
            cloud = new ScanCloud();

            foreach (ScanSegment segment in segments)
            {
                // Calculate segment relative position against latest odometry-measured position
                Vector3 pose = segment.Pose - odometryPose;

                foreach (Ray r in segment.Rays)
                {
                    Vector2 hitPoint = new Vector2()
                    {
                        X = pose.X + r.Radius * MathF.Cos(r.Angle + pose.Z),
                        Y = pose.Y + r.Radius * MathF.Sin(r.Angle + pose.Z),
                    };

                    cloud.Points.Add(hitPoint);
                }
            }
        }

        /// <summary>
        /// Calculate "distance" of cloud at determined pose
        /// </summary>
        /// <param name="cloud">Cloud of points</param>
        /// <param name="pose">Pose of cloud</param>
        /// <returns></returns>
        private int CalculateDistance(ScanCloud cloud, Vector3 pose)
        {
            return CalculateDistanceSISD(cloud, pose);
        }

        /// <summary>
        /// Calculate "distance" of cloud at determined pose
        /// </summary>
        /// <param name="cloud">Cloud of points</param>
        /// <param name="pose">Pose of cloud</param>
        /// <returns></returns>
        private int CalculateDistanceSISD(ScanCloud cloud, Vector3 pose)
        {
            int nb_points = 0;
            long sum = 0;

            // Pre-calculate centerpoint with additional 0.5 to get rounding effect
            float px = pose.X * HoleMap.Scale + 0.5f;
            float py = pose.Y * HoleMap.Scale + 0.5f;
            float c = MathF.Cos(pose.Z) * HoleMap.Scale;
            float s = MathF.Sin(pose.Z) * HoleMap.Scale;

            // Translate and rotate scan to robot position and compute the "distance"
            for (int i = 0; i < cloud.Points.Count; i++)
            {
                int x = (int)(px + c * cloud.Points[i].X - s * cloud.Points[i].Y);
                int y = (int)(py + s * cloud.Points[i].X + c * cloud.Points[i].Y);

                // Check boundaries
                if ((x >= 0) && (x < HoleMap.Size) && (y >= 0) && (y < HoleMap.Size))
                {
                    sum += HoleMap.Pixels[y * HoleMap.Size + x];
                    nb_points++;
                }
            }

            if (nb_points > 0)
            {
                return (int)((sum * 1024) / cloud.Points.Count);
            }
            else
            {
                return int.MaxValue;
            }
        }

        /// <summary>
        /// Calculate "distance" of cloud at determined pose
        /// TODO - It's actually slower than SISD. Need more parallelism.
        /// </summary>
        /// <param name="cloud">Cloud of points</param>
        /// <param name="pose">Pose of cloud</param>
        /// <returns></returns>
        private int CalculateDistanceSSE41(ScanCloud cloud, Vector3 pose)
        {
            int nb_points = 0;
            long sum = 0;

            float px = pose.X * HoleMap.Scale;
            float py = pose.Y * HoleMap.Scale;
            float c = MathF.Cos(pose.Z) * HoleMap.Scale;
            float s = MathF.Sin(pose.Z) * HoleMap.Scale;

            Vector128<float> sincos = Vector128.Create(c, -s, s, c);
            Vector128<float> posxy = Vector128.Create(px, py, px, py);

            // Translate and rotate scan to robot position and compute the "distance"
            for (int i = 0; i < cloud.Points.Count; i++)
            {
                Vector128<float> xy = Vector128.Create(cloud.Points[i].X, cloud.Points[i].Y, cloud.Points[i].X, cloud.Points[i].Y);
                xy = Sse41.Multiply(sincos, xy);
                xy = Sse41.HorizontalAdd(xy, xy);
                xy = Sse41.Add(xy, posxy);
                xy = Sse41.RoundToNearestInteger(xy);

                int x = (int)xy.GetElement(0);
                int y = (int)xy.GetElement(1);

                // Check boundaries
                if ((x >= 0) && (x < HoleMap.Size) && (y >= 0) && (y < HoleMap.Size))
                {
                    sum += HoleMap.Pixels[y * HoleMap.Size + x];
                    nb_points++;
                }
            }

            if (nb_points > 0)
            {
                return (int)((sum * 1024) / cloud.Points.Count);
            }
            else
            {
                return int.MaxValue;
            }
        }

        /// <summary>
        /// Clip ray
        /// </summary>
        /// <param name="size">Map size</param>
        /// <param name="xyc"></param>
        /// <param name="yxc"></param>
        /// <param name="xy"></param>
        /// <param name="yx"></param>
        /// <returns></returns>
        private static bool ClipRay(int size, ref int xyc, ref int yxc, int xy, int yx)
        {
            if (xyc < 0)
            {
                if (xyc == xy)
                {
                    return false;
                }

                yxc += (yxc - yx) * (-xyc) / (xyc - xy);
                xyc = 0;
            }

            if (xyc >= size)
            {
                if (xyc == xy)
                {
                    return false;
                }

                yxc += (yxc - yx) * (size - 1 - xyc) / (xyc - xy);
                xyc = size - 1;
            }

            return true;
        }

        /// <summary>
        /// Draw laser ray on hole map
        /// </summary>
        /// <param name="map">Map</param>
        /// <param name="x1">Start x</param>
        /// <param name="y1">Start y</param>
        /// <param name="x2">Obstacle x</param>
        /// <param name="y2">Obstacle y</param>
        /// <param name="xp"></param>
        /// <param name="yp"></param>
        /// <param name="value"></param>
        /// <param name="alpha"></param>
        private void DrawLaserRayOnHoleMap(int x1, int y1, int x2, int y2, int xp, int yp, int value, int alpha)
        {
            int x2c = x2;
            int y2c = y2;

            // Clipping of the laser beam
            if (!ClipRay(HoleMap.Size, ref x2c, ref y2c, x1, y1)) return;
            if (!ClipRay(HoleMap.Size, ref y2c, ref x2c, y1, x1)) return;

            int dx = Math.Abs(x2 - x1);
            int dy = Math.Abs(y2 - y1);
            int dxc = Math.Abs(x2c - x1);
            int dyc = Math.Abs(y2c - y1);
            int incptrx = Math.Sign(x2 - x1);
            int incptry = Math.Sign(y2 - y1) * HoleMap.Size;
            int sincv = Math.Sign(value - TS_NO_OBSTACLE);
            int derrorv;

            if (dx > dy)
            {
                derrorv = Math.Abs(xp - x2);
            }
            else
            {
                (dx, _) = (dy, dx);
                (dxc, dyc) = (dyc, dxc);
                (incptrx, incptry) = (incptry, incptrx);
                derrorv = Math.Abs(yp - y2);
            }

            if (derrorv == 0)
            {
                return;
            }

            int error = 2 * dyc - dxc;
            int horiz = 2 * dyc;
            int diago = 2 * (dyc - dxc);
            int errorv = derrorv / 2;
            int incv = (value - TS_NO_OBSTACLE) / derrorv;
            int incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;

            int ptr = y1 * HoleMap.Size + x1;
            int pixval = TS_NO_OBSTACLE;

            for (int x = 0; x <= dxc; x++, ptr += incptrx)
            {
                if (x > dx - 2 * derrorv)
                {
                    if (x <= dx - derrorv)
                    {
                        pixval += incv;
                        errorv += incerrorv;
                        if (errorv > derrorv)
                        {
                            pixval += sincv;
                            errorv -= derrorv;
                        }
                    }
                    else
                    {
                        pixval -= incv;
                        errorv -= incerrorv;
                        if (errorv < 0)
                        {
                            pixval -= sincv;
                            errorv += derrorv;
                        }
                    }
                }

                // Integration into the map
                HoleMap.Pixels[ptr] = (ushort)(((256 - alpha) * HoleMap.Pixels[ptr] + alpha * pixval) >> 8);

                if (error > 0)
                {
                    ptr += incptry;
                    error += diago;
                }
                else
                {
                    error += horiz;
                }
            }
        }

        /// <summary>
        /// Draw laser ray on obstacle map
        /// From:
        /// https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C
        /// But modified it
        /// </summary>
        /// <param name="noHitMap">No-hit map</param>
        /// <param name="x1">Start x</param>
        /// <param name="y1">Start y</param>
        /// <param name="x2">Obstacle x</param>
        /// <param name="y2">Obstacle y</param>
        private void DrawLaserRayOnObstacleMap(bool[,] noHitMap, int x1, int y1, int x2, int y2)
        {
            int dx = Math.Abs(x2 - x1), sx = Math.Sign(x2 - x1);
            int dy = Math.Abs(y2 - y1), sy = Math.Sign(y2 - y1);
            int err = (dx > dy ? dx : -dy) / 2, e2;

            while (true)
            {
                // Outside of map ?
                if ((x1 < 0) || (x1 >= ObstacleMap.Size) ||
                    (y1 < 0) || (y1 >= ObstacleMap.Size))
                {
                    break;
                }
                // Hit point ?
                else if (x1 == x2 && y1 == y2)
                {
                    // Add hit
                    if (ObstacleMap.Pixels[y1, x1] < MaxObstacleHits)
                    {
                        ObstacleMap.Pixels[y1, x1]++;
                    }
                    break;
                }
                else
                {
                    // Mark no hits
                    noHitMap[y1, x1] = true;
                }

                e2 = err;
                if (e2 > -dx) { err -= dy; x1 += sx; }
                if (e2 < dy) { err += dx; y1 += sy; }
            }
        }

        /// <summary>
        /// Update hole map with lidar scan points cloud
        /// </summary>
        /// <param name="cloud">Scan points cloud</param>
        private void UpdateHoleMap(ScanCloud cloud)
        {
            // Pre-calculate centerpoint with additional 0.5 to get rounding effect
            float px = Pose.X * HoleMap.Scale + 0.5f;
            float py = Pose.Y * HoleMap.Scale + 0.5f;
            float c = MathF.Cos(Pose.Z) * HoleMap.Scale;
            float s = MathF.Sin(Pose.Z) * HoleMap.Scale;

            // Line start
            int x1 = (int)px;
            int y1 = (int)py;

            // Boundary check
            if (x1 < 0 || x1 >= HoleMap.Size || y1 < 0 || y1 >= HoleMap.Size)
            {
                return; // Robot is out of map
            }
            
            // TODO Use cloud lines to draw map ?

            // Translate and rotate scan to robot position
            foreach (Vector2 p in cloud.Points)
            {
                float x2p = c * p.X - s * p.Y;
                float y2p = s * p.X + c * p.Y;
                int xp = (int)(px + x2p);
                int yp = (int)(py + y2p);

                float dist = MathF.Sqrt(x2p * x2p + y2p * y2p);
                float add = HoleWidth * HoleMap.Scale / 2.0f / dist;

                x2p *= (1.0f + add);
                y2p *= (1.0f + add);
                int x2 = (int)(px + x2p);
                int y2 = (int)(py + y2p);

                DrawLaserRayOnHoleMap(x1, y1, x2, y2, xp, yp, TS_OBSTACLE, Quality);
            }
        }

        /// <summary>
        /// Update obstacle map with lidar scan points cloud
        /// </summary>
        /// <param name="cloud">Scan points cloud</param>
        private void UpdateObstacleMap(ScanCloud cloud)
        {
            ArrayEx.Fill(noHitMap, false);

            // Pre-calculate centerpoint with additional 0.5 to get rounding effect
            float px = Pose.X * ObstacleMap.Scale + 0.5f;
            float py = Pose.Y * ObstacleMap.Scale + 0.5f;
            float c = MathF.Cos(Pose.Z) * ObstacleMap.Scale;
            float s = MathF.Sin(Pose.Z) * ObstacleMap.Scale;

            // TODO Use cloud lines to draw map ?

            // Line start
            int x1 = (int)px;
            int y1 = (int)py;

            // Boundary check
            if (x1 < 0 || x1 >= ObstacleMap.Size || y1 < 0 || y1 >= ObstacleMap.Size)
            {
                return; // Robot is out of map
            }

            // Draw rays
            foreach (Vector2 p in cloud.Points)
            {
                // Translate and rotate scan to robot position
                int x2 = (int)(px + c * p.X - s * p.Y);
                int y2 = (int)(py + s * p.X + c * p.Y);

                // Draw obstacle map
                DrawLaserRayOnObstacleMap(noHitMap, x1, y1, x2, y2);
            }

            // TODO Far small objects disappear too easily... do some tricks

            // Deal with no hit pixels
            for (int y = 0; y < ObstacleMap.Size; y++)
            {
                for (int x = 0; x < ObstacleMap.Size; x++)
                {
                    if (noHitMap[y, x])
                    {
                        if (ObstacleMap.Pixels[y, x] < 0)
                        {
                            ObstacleMap.Pixels[y, x]++;
                        }
                        else if (ObstacleMap.Pixels[y, x] > 0)
                        {
                            ObstacleMap.Pixels[y, x]--;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Fill random number queues which are used in parallel search.
        /// </summary>
        /// <param name="index">Thread index</param>
        private void FillRandomQueues(int index)
        {
            // Make sure there are enough random numbers in queue
            // XY queue has to be twice as large as theta queue
            while (randomQueuesXY[index].Count < SearchIterationsPerThread * 2)
            {
                randomQueuesXY[index].Enqueue(samplerXY.Sample());
            }

            while (randomQueuesTheta[index].Count < SearchIterationsPerThread)
            {
                randomQueuesTheta[index].Enqueue(samplerTheta.Sample());
            }
        }

        /// <summary>
        /// Search for best robot pose in monto-carlo method
        /// </summary>
        /// <param name="cloud">Scan points cloud</param>
        /// <param name="searchPose">Search start position</param>
        /// <param name="randomXY">Queue of random XY standard deviation numbers</param>
        /// <param name="randomTheta">Queue of random Theta standard deviation numbers</param>
        /// <param name="iterations">Number of search iterations</param>
        /// <param name="distance">Best pose distance value (the lower the better)</param>
        /// <returns>Best found pose</returns>
        private Vector3 MonteCarloSearch(ScanCloud cloud, Vector3 searchPose, Func<float> randomXY, Func<float> randomTheta, int iterations, out int distance)
        {
            Vector3 bestPose = searchPose;
            int currentDistance = CalculateDistance(cloud, searchPose);
            int bestDistance = currentDistance;

            for (int counter = 0; counter < iterations; counter++)
            {
                // Create new random position
                Vector3 currentpose = new Vector3()
                {
                    X = searchPose.X + randomXY(),
                    Y = searchPose.Y + randomXY(),
                    Z = searchPose.Z + randomTheta()
                };

                // Calculate distance at that position
                currentDistance = CalculateDistance(cloud, currentpose);

                // Is it the best ?
                if (currentDistance < bestDistance)
                {
                    bestDistance = currentDistance;
                    bestPose = currentpose;
                }
            }

            distance = bestDistance;
            return bestPose;
        }

        /// <summary>
        /// Search for best robot pose in monto-carlo method
        /// </summary>
        /// <param name="cloud">Scan points cloud</param>
        /// <param name="searchPose">Search start position</param>
        /// <param name="distance">Best pose distance value (the lower the better)</param>
        /// <returns>Best found pose</returns>
        private Vector3 SingleMonteCarloSearch(ScanCloud cloud, Vector3 searchPose, out int distance)
        {
            return MonteCarloSearch(cloud, searchPose, () => samplerXY.Sample(), () => samplerTheta.Sample(), SearchIterationsPerThread, out distance);
        }

        /// <summary>
        /// Search for best robot position in monto-carlo method
        /// </summary>
        /// <param name="cloud">Scan points cloud</param>
        /// <param name="searchPose">Search start position</param>
        /// <param name="distance">Best pose distance value (the lower the better)</param>
        /// <returns>Best found pose</returns>
        private Vector3 ParallelMonteCarloSearch(ScanCloud cloud, Vector3 searchPose, out int distance)
        {
            int[] distances = new int[NumSearchThreads];
            Vector3[] poses = new Vector3[NumSearchThreads];

            // Let all worker threads find best position
            worker.Work(index =>
            {
                poses[index] = MonteCarloSearch(
                    cloud,
                    searchPose,
                    () => randomQueuesXY[index].Dequeue(),
                    () => randomQueuesTheta[index].Dequeue(),
                    SearchIterationsPerThread,
                    out distances[index]);
            }, true);

            // Queue a task to fill random number queues for next search loop
            worker.Work(FillRandomQueues, false);

            // Find best pose of out all worker results
            int bestDistance = int.MaxValue;
            Vector3 bestPose = searchPose;

            for (int i = 0; i < NumSearchThreads; i++)
            {
                if (distances[i] < bestDistance)
                {
                    bestDistance = distances[i];
                    bestPose = poses[i];
                }
            }

            // Return best pose and "distance"
            distance = bestDistance;
            return bestPose;
        }

        /// <summary>
        /// Update map and calculate robot position based on odometry and lidar scan data with SLAM method
        /// It takes robot movement into account - there's no single 360 degree scan to feed into here,
        /// instead there are scan segments, each with own pose.
        /// </summary>
        public void Update(List<ScanSegment> segments)
        {
            Vector3 odoPose = segments.Last().Pose;
            Vector3 newPose;

            // Convert scan segments to cloud
            ScanSegmentsToCloud(segments, odoPose, out ScanCloud cloud);

            // Do pose search from map ?
            if (scanCount >= PositionSearchBeginning)
            {
                Vector3 searchPose = Pose + (odoPose - lastOdometryPose);

                if (worker != null)
                {
                    newPose = ParallelMonteCarloSearch(cloud, searchPose, out int _);
                }
                else
                {
                    newPose = SingleMonteCarloSearch(cloud, searchPose, out int _);
                }
            }
            else
            {
                scanCount++;
                newPose = odoPose;
            }

            lastOdometryPose = odoPose;
            newPose.Z = MathEx.NormalizeAngle(newPose.Z);
            Pose = newPose;

            // Update maps
            UpdateHoleMap(cloud);
            UpdateObstacleMap(cloud);
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
