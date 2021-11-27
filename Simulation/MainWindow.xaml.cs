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
using Microsoft.Extensions.Logging;
using BaseSLAM;
using CoreSLAM;
using HectorSLAM;
using HectorSLAM.Main;
using HectorSLAM.Map;

namespace Simulation
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Constnts
        private const int numScanPoints = 400;     // Scan points per revolution
        private const float scanPerSecond = 17.0f; // Scan / second
        private const float maxScanDist = 40.0f;   // Meters
        private const float measureError = 0.02f;  // Meters
        private const int scanPeriod = (int)(1000.0 / scanPerSecond); // Milliseconds

        // Objects
        private readonly BufferedLogger bufferedLogger;
        private readonly Field field = new Field();
        private readonly ScaleTransform fieldScale;
        private readonly DispatcherTimer drawTimer;
        private Vector3 startPose;
        private Vector3 lidarPose;
        private readonly CoreSLAMProcessor coreSlam;
        private readonly HectorSLAMProcessor hectorSlam;
        private readonly Thread lidarThread;
        private bool doReset;
        private bool isRunning;

        /// <summary>
        /// Constructor
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            // Logger
            bufferedLogger = new BufferedLogger();

            // Initial poses
            startPose = new Vector3(20.0f, 20.0f, 0.0f);
            lidarPose = startPose;

            // CoreSLAM
            coreSlam = new CoreSLAMProcessor(40.0f, 256, 64, startPose, 0.1f, MathEx.DegToRad(10), 1000, 4)
            {
                HoleWidth = 2.0f
            };

            // HectorSLAM
            int hsMapSide = 400;
            hectorSlam = new HectorSLAMProcessor(40.0f / hsMapSide, new System.Drawing.Point(hsMapSide, hsMapSide), startPose, 4, 4, bufferedLogger)
            {
                MinDistanceDiffForMapUpdate = 0.4f,
                MinAngleDiffForMapUpdate = MathEx.DegToRad(8)
            };

            // Set estimate iteransions counts for each map layer
            hectorSlam.MapRep.Maps[0].EstimateIterations = 7;
            hectorSlam.MapRep.Maps[1].EstimateIterations = 4;
            hectorSlam.MapRep.Maps[2].EstimateIterations = 4;
            hectorSlam.MapRep.Maps[3].EstimateIterations = 4;

            // Create map layer selection combobox
            for (int m = 0; m < hectorSlam.MapRep.NumLevels; m++)
            {
                VisibleHectorSLAMLayerComboBox.Items.Add($"#{m + 1} - {hectorSlam.MapRep.Maps[m].Dimensions.X}x{hectorSlam.MapRep.Maps[m].Dimensions.Y}");
            }

            VisibleHectorSLAMLayerComboBox.SelectedIndex = 0;

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
            hectorSlam.Dispose();
        }

        /// <summary>
        /// Scan
        /// </summary>
        private void Scan()
        {
            int loops = 0;
            bool hsWrong = false;
            
            while (isRunning)
            {
                if (doReset)
                {
                    coreSlam.Reset();
                    hectorSlam.Reset();
                    lidarPose = startPose;
                    loops = 0;

                    doReset = false;
                    hsWrong = false;
                }

                var sw = Stopwatch.StartNew();

                var snapPose = lidarPose;
                bufferedLogger.LogInformation($"Real pose {snapPose.ToPoseString()}");

                ScanSegments(snapPose, coreSlam.Pose, out List<ScanSegment> scanSegments);
                coreSlam.Update(scanSegments);

                ScanCloud scanCloud = new ScanCloud()
                {
                    Pose = Vector3.Zero
                };

                foreach (ScanSegment seg in scanSegments)
                {
                    foreach (Ray ray in seg.Rays)
                    {
                        scanCloud.Points.Add(new Vector2()
                        {
                            X = ray.Radius * MathF.Cos(ray.Angle),
                            Y = ray.Radius * MathF.Sin(ray.Angle),
                        });
                    }
                }

                hectorSlam.Update(scanCloud, hectorSlam.MatchPose, loops < 10);

                // Detect first large difference in hector SLAM
                if (!hsWrong)
                {
                    Vector2 linDiff = hectorSlam.MatchPose.ToVector2() - snapPose.ToVector2();
                    float distDiff = linDiff.Length();
                    float angDiff = MathEx.RadToDeg(MathEx.RadDiff(hectorSlam.MatchPose.Z, snapPose.Z));
                    if ((distDiff > 1.0f) || (Math.Abs(angDiff) > 10.0f))
                    {
                        Debug.WriteLine(string.Empty);
                        Debug.WriteLine($"{DateTime.Now:mm:ss:ffff} - HectorSLAM large difference");
                        Debug.WriteLine($"  Distance {distDiff:f2}m, angular: {angDiff:f2}°, linear: {linDiff.X:f2}x{linDiff.Y:f2}m");
                        bufferedLogger.Items.TakeLast(30).ForEach(i => Debug.WriteLine($"  {i}"));
                        Debug.WriteLine(string.Empty);
                        hsWrong = true;
                    }
                }

                // Clear log buffer once in a while
                if (bufferedLogger.Items.Count > 130)
                {
                    bufferedLogger.Items.RemoveRange(0, 100);
                }

                // Ensure periodicity
                Thread.Sleep((int)Math.Max(0, (long)scanPeriod - sw.ElapsedMilliseconds));

                // Count loops
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

            // What SLAM map to show ?
            switch (VisibleSLAMComboBox.SelectedIndex)
            {
                // CoreSLAM ?
                case 0:

                    // Construct hole map image
                    WriteableBitmap holeMapBitmap = new WriteableBitmap(coreSlam.HoleMap.Size, coreSlam.HoleMap.Size, 96, 96, PixelFormats.Gray16, null);
                    Int32Rect csRect = new Int32Rect(0, 0, coreSlam.HoleMap.Size, coreSlam.HoleMap.Size);
                    holeMapBitmap.WritePixels(csRect, coreSlam.HoleMap.Pixels, holeMapBitmap.BackBufferStride, 0);

                    Image holeMapImage = new Image()
                    {
                        Source = holeMapBitmap,
                        Width = coreSlam.PhysicalMapSize,
                        Height = coreSlam.PhysicalMapSize,
                    };

                    DrawArea.Children.Add(holeMapImage);
                    break;

                // HectorSLAM ?
                case 1:

                    // Construct occupancy map image
                    OccGridMap map = hectorSlam.MapRep.Maps[VisibleHectorSLAMLayerComboBox.SelectedIndex];
                    WriteableBitmap occupancyMapBitmap = new WriteableBitmap(map.Dimensions.X, map.Dimensions.Y, 96, 96, PixelFormats.Gray8, null);
                    Int32Rect hsRect = new Int32Rect(0, 0, map.Dimensions.X, map.Dimensions.Y);
                    occupancyMapBitmap.WritePixels(hsRect, map.GetBitmapData(), occupancyMapBitmap.PixelWidth, 0);

                    Image occMapImage = new Image()
                    {
                        Source = occupancyMapBitmap,
                        Width = map.Properties.PhysicalSize.X,
                        Height = map.Properties.PhysicalSize.Y,
                    };

                    DrawArea.Children.Add(occMapImage);
                    break;
            }

            // Draw real field edges
            DrawField();

            // Draw poses
            DrawPose(lidarPose, 0.2f, Colors.Red);
            DrawPose(coreSlam.Pose, 0.2f, Colors.Blue);
            DrawPose(hectorSlam.MatchPose, 0.2f, Colors.Green);

            // Update labels
            RealPoseLabel.Text = lidarPose.ToPoseString();
            CoreSLAMPoseLabel.Text = coreSlam.Pose.ToPoseString();
            HectorSLAMPoseLabel.Text = hectorSlam.MatchPose.ToPoseString();
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
        /// "Draw" pose.
        /// </summary>
        /// <param name="pose">Pose</param>
        /// <param name="radius">Radius</param>
        /// <param name="color">Color</param>
        private void DrawPose(Vector3 pose, float radius, Color color)
        {
            Ellipse circle = new Ellipse()
            {
                Fill = new SolidColorBrush(color),
                Width = radius * 2,
                Height = radius * 2
            };

            Canvas.SetLeft(circle, pose.X - radius);
            Canvas.SetTop(circle, pose.Y - radius);

            Line line = new Line()
            {
                Stroke = new SolidColorBrush(color),
                StrokeThickness = radius / 4.0f,
                X1 = pose.X,
                Y1 = pose.Y,
                X2 = pose.X + radius * 3 * MathF.Cos(pose.Z),
                Y2 = pose.Y + radius * 3 * MathF.Sin(pose.Z),
            };

            DrawArea.Children.Add(line);
            DrawArea.Children.Add(circle);
        }

        /// <summary>
        /// Scan segments
        /// </summary>
        /// <param name="realPose">Real pose (to use for scanning)</param>
        /// <param name="estimatedPose">Estimated pose (to use to store in segments)</param>
        /// <param name="segments"></param>
        private void ScanSegments(Vector3 realPose, Vector3 estimatedPose, out List<ScanSegment> segments)
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
                float lidarAngle = angle + realPose.Z;

                if (field.RayTrace(realPose.ToVector2(), lidarAngle, maxScanDist, out float hit))
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

            if (e.RightButton == MouseButtonState.Pressed)
            {
                UpdateLidarViewDirection(e.GetPosition(DrawArea));
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
            
            if (e.RightButton == MouseButtonState.Pressed)
            {
                UpdateLidarViewDirection(e.GetPosition(DrawArea));
            }
        }

        /// <summary>
        /// Update lidar position
        /// </summary>
        /// <param name="p">New position point</param>
        private void UpdateLidarPosition(Point p)
        {
            var newLidarPos = new Vector2((float)p.X, (float)p.Y);
            lidarPose = new Vector3(newLidarPos, lidarPose.Z);
        }

        /// <summary>
        /// Update lidar view direction
        /// </summary>
        /// <param name="p">New view direction</param>
        private void UpdateLidarViewDirection(Point p)
        {
            var dirPos = new Vector2((float)p.X, (float)p.Y);
            Vector2 diff = dirPos - lidarPose.ToVector2();
            float angle = MathF.Atan2(diff.Y, diff.X);
            lidarPose = new Vector3(lidarPose.ToVector2(), angle);
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
