using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using BaseSLAM;
using CoreSLAM;
using HectorSLAM;
using HectorSLAM.Main;
using HectorSLAM.Map;
using HectorSLAM.Scan;

namespace Simulation
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Constnts
        private const int numScanPoints = 400;     // Scan points per revolution
        private const float scanPerSecond = 7.0f;  // Scan / second
        private const float maxScanDist = 40.0f;   // Meters
        private const float measureError = 0.02f;  // Meters
        private const int scanPeriod = (int)(1000.0 / scanPerSecond); // Milliseconds

        // Objects
        private readonly Field field = new Field();
        private readonly ScaleTransform fieldScale;
        private readonly DispatcherTimer drawTimer;
        private Vector2 startPos;
        private Vector2 lidarPos;
        private readonly CoreSLAMProcessor coreSlam;
        private readonly HectorSLAMProcessor hectorSlam;
        private readonly Thread lidarThread;
        private readonly WriteableBitmap holeMapBitmap;
        private readonly WriteableBitmap occupancyMapBitmap;
        private bool doReset;
        private bool isRunning;

        /// <summary>
        /// Constructor
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            // Create SLAM
            startPos = new Vector2(20.0f, 20.0f);
            lidarPos = startPos;

            coreSlam = new CoreSLAMProcessor(40.0f, 256, 64, new Vector3(lidarPos.X, lidarPos.Y, 0.0f), 4)
            {
                HoleWidth = 2.0f,
                SigmaXY = 1.0f
            };

            holeMapBitmap = new WriteableBitmap(coreSlam.HoleMap.Size, coreSlam.HoleMap.Size, 96, 96, PixelFormats.Gray16, null);

            // Important tip: the robot movement speed cannot be greater than a coarsest map pixel per scan.
            hectorSlam = new HectorSLAMProcessor(40.0f / 400, 400, 400, Vector2.Zero, 4)
            {
                LastScanMatchPose = startPos.ToVector3(),
                LastMapUpdatePose = startPos.ToVector3(),
                MinDistanceDiffForMapUpdate = 0.4f,
                MinAngleDiffForMapUpdate = 0.15f
            };

            // Create field
            field.CreateDefaultField(30.0f, new Vector2(5.0f, 5.0f));

            // Set render transformation (scaling)
            fieldScale = new ScaleTransform(16, 16);
            DrawArea.RenderTransform = fieldScale;

            // Start periodic draw function
            drawTimer = new DispatcherTimer();
            drawTimer.Tick += (s, e) => Draw();
            drawTimer.Interval = TimeSpan.FromMilliseconds(20); // 50 fps
            drawTimer.Start();

            // Start scan timer in another thread
            lidarThread = new Thread(new ThreadStart(Scan))
            {
                Name = "Lidar"
            };

            isRunning = true;
            lidarThread.Start();
        }

        /// <summary>
        /// Window unloaded.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Window_Closed(object sender, EventArgs e)
        {
            isRunning = false;
            drawTimer.Stop();
            lidarThread.Join();
            coreSlam.Dispose();
        }

        /// <summary>
        /// Scan
        /// </summary>
        private void Scan()
        {
            Stopwatch sw = new Stopwatch();
            int loops = 0;
            

            while (isRunning)
            {
                if (doReset)
                {
                    coreSlam.Reset();
                    hectorSlam.Reset();
                    lidarPos = startPos;
                    doReset = false;
                }

                Vector2 lpos = lidarPos;

                sw.Restart();

                ScanSegments(lpos, coreSlam.Pose, out List<ScanSegment> scanSegments);
                coreSlam.Update(scanSegments);

                //Vector3 lhp = lpos.ToVector3();
                Vector3 lhp = hectorSlam.LastScanMatchPose;

                DataContainer dt = new DataContainer
                {
                    Origin = new Vector2(0, 0)
                };

                foreach (ScanSegment seg in scanSegments)
                {
                    foreach (Ray ray in seg.Rays)
                    {
                        dt.Add(new Vector2()
                        {
                            X = ray.Radius * MathF.Cos(ray.Angle),
                            Y = ray.Radius * MathF.Sin(ray.Angle),
                        });
                    }
                }

                hectorSlam.Update(dt, lhp, loops < 10);

                // Ensure periodicity
                Thread.Sleep((int)Math.Max(0, (long)scanPeriod - sw.ElapsedMilliseconds));

                loops++;
            }
        }

        /// <summary>
        /// Draw field and everything
        /// </summary>
        private void Draw()
        {
            DrawArea.Children.Clear();
            DrawBackground();

            // Construct hole map image
            Int32Rect rect = new Int32Rect(0, 0, coreSlam.HoleMap.Size, coreSlam.HoleMap.Size);
            holeMapBitmap.WritePixels(rect, coreSlam.HoleMap.Pixels, holeMapBitmap.BackBufferStride, 0);

            Image holeMapImage = new Image()
            {
                Source = holeMapBitmap,
                Width = coreSlam.PhysicalMapSize,
                Height = coreSlam.PhysicalMapSize,
            };

            //DrawArea.Children.Add(holeMapImage);

            // Construct occupancy map image
            
            var map = hectorSlam.MapRep.GetGridMap(hectorSlam.MapRep.NumLevels - 1);
            int div = 1;
            
            var occupancyMapBitmap = new WriteableBitmap(map.Dimensions.X / div, map.Dimensions.Y / div, 96, 96, PixelFormats.Gray8, null);

            occupancyMapBitmap.Lock();

            for (int y = 0; y < map.Dimensions.Y / div; y++)
            {
                for (int x = 0; x < map.Dimensions.X / div; x++)
                {
                    Int32Rect pr = new Int32Rect(x, y, 1, 1);
                    byte[] data = new byte[1];
                    ICell cell = map.GetCell(x * div, y * div);

                    if (cell.IsFree)
                    {
                        data[0] = 255;
                    }
                    else if (cell.IsOccupied)
                    {
                        data[0] = 0;
                    }
                    else
                    {
                        data[0] = 127;
                    }

                    occupancyMapBitmap.WritePixels(pr, data, occupancyMapBitmap.PixelWidth, 0);
                }
            }

            occupancyMapBitmap.Unlock();

            Image occMapImage = new Image()
            {
                Source = occupancyMapBitmap,
                Width = map.Properties.PhysicalSize.X,
                Height = map.Properties.PhysicalSize.Y,
            };

            RenderOptions.SetBitmapScalingMode(occMapImage, BitmapScalingMode.Linear);

            DrawArea.Children.Add(occMapImage);

            // Draw field edges
            DrawField();

            // Draw positions
            DrawCircle(lidarPos, 0.2f, Colors.Blue);
            DrawCircle(coreSlam.Pose.ToVector2(), 0.2f, Colors.Red);
            DrawCircle(hectorSlam.LastMapUpdatePose.ToVector2(), 0.2f, Colors.Yellow);
            DrawCircle(hectorSlam.LastScanMatchPose.ToVector2(), 0.2f, Colors.Green);

            // Update labels
            RealPosLabel.Text = $"Real position: {lidarPos.X:f2} x {lidarPos.Y:f2}";
            EstimatedPosLabel.Text = $"Estimated position: {coreSlam.Pose.X:f2} x {coreSlam.Pose.Y:f2}";
        }

        /// <summary>
        /// "Draw" background. Need some object on canvas to get mouse events.
        /// </summary>
        private void DrawBackground()
        {
            // Have 10x10 km rectangle.
            var bg = new Rectangle()
            {
                Fill = Brushes.White,
                Width = 10000.0f,
                Height = 10000.0f,
            };
            
            DrawArea.Children.Add(bg);
        }

        /// <summary>
        /// "Draw" field edges.
        /// </summary>
        private void DrawField()
        {
            foreach ((Vector2, Vector2) edge in field.GetEdges())
            {
                Line line = new Line()
                {
                    X1 = edge.Item1.X,
                    Y1 = edge.Item1.Y,
                    X2 = edge.Item2.X,
                    Y2 = edge.Item2.Y,
                    Stroke = Brushes.Blue,
                    StrokeThickness = 0.1f
                };

                line.Opacity = 0.5f;

                DrawArea.Children.Add(line);
            }
        }

        /// <summary>
        /// "Draw" scan segment.
        /// </summary>
        /// <param name="segment">Scan segment</param>
        private void DrawScan(ScanSegment segment)
        {
            foreach (Ray ray in segment.Rays)
            {
                Vector2 endPos = new Vector2(
                    ray.Radius * MathF.Cos(ray.Angle),
                    ray.Radius * MathF.Sin(ray.Angle));

                Line line = new Line()
                {
                    X1 = segment.Pose.X,
                    Y1 = segment.Pose.Y,
                    X2 = segment.Pose.X + endPos.X,
                    Y2 = segment.Pose.Y + endPos.Y,
                    Stroke = Brushes.Red,
                    StrokeThickness = 0.05
                };

                DrawArea.Children.Add(line);
            }
        }

        /// <summary>
        /// "Draw" circle.
        /// </summary>
        /// <param name="pos">Position</param>
        /// <param name="radius">Radius</param>
        /// <param name="color">Color</param>
        private void DrawCircle(Vector2 pos, float radius, Color color)
        {
            Ellipse circle = new Ellipse()
            {
                Fill = new SolidColorBrush(color),
                Width = radius * 2,
                Height = radius * 2
            };

            Canvas.SetLeft(circle, pos.X - radius);
            Canvas.SetTop(circle, pos.Y - radius);

            DrawArea.Children.Add(circle);
        }

        /// <summary>
        /// Scan segments
        /// </summary>
        /// <param name="realPos">Real position (to use for scanning)</param>
        /// <param name="estimatedPose">Estimated pose (to use to store in segments)</param>
        /// <param name="segments"></param>
        private void ScanSegments(Vector2 realPos, Vector3 estimatedPose, out List<ScanSegment> segments)
        {
            Random rnd = new Random();
            float scanAngle = (MathF.PI * 2) / numScanPoints;

            ScanSegment scanSegment = new ScanSegment()
            {
                Pose = estimatedPose,
                IsLast = true
            };

            for (float angle = 0.0f; angle < MathF.PI * 2; angle += scanAngle)
            {
                if (field.RayTrace(realPos, angle, maxScanDist, out float hit))
                {
                    hit += ((float)rnd.Next(-100, 100) / 100.0f) * measureError;

                    scanSegment.Rays.Add(new Ray(angle, hit));
                }
            }

            segments = new List<ScanSegment>()
            {
                scanSegment
            };
        }

        /// <summary>
        /// Mouse move event
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Field_MouseMove(object sender, MouseEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                UpdateLidarPosition(e.GetPosition(DrawArea));
            }
        }

        /// <summary>
        /// Mouse down event
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Field_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                UpdateLidarPosition(e.GetPosition(DrawArea));
            }
        }

        /// <summary>
        /// Update lidar position
        /// </summary>
        /// <param name="p"></param>
        private void UpdateLidarPosition(Point p)
        {
            lidarPos = new Vector2((float)p.X, (float)p.Y);
        }

        /// <summary>
        /// Mouse wheel action on canvas.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Field_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            double delta = Math.Sign(e.Delta) * 1.0;
            double scale = Math.Max(1.0, Math.Min(100.0, fieldScale.ScaleX + delta));

            fieldScale.ScaleX = scale;
            fieldScale.ScaleY = scale;
        }

        /// <summary>
        /// Reset button click
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            doReset = true;
        }
    }
}
