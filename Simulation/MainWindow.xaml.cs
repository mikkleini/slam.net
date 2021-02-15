using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
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
using BaseSLAM;
using CoreSLAM;

namespace Simulation
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Constnts
        private const int numScanPoints = 600;     // Scan points per revolution        
        private const float scanPerSecond = 7.0f;  // Scan / second
        private const float maxScanDist = 40.0f;   // Meters

        // Objects
        private Field field = new Field();
        private Vector2 lidarPos = new Vector2(0, 0);
        private ScaleTransform fieldScale = new ScaleTransform(20, 20);
        private System.Windows.Threading.DispatcherTimer scanTimer;
        
        /// <summary>
        /// Constructor
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            // Create field
            field.CreateDefaultField();

            // Set render transformation (scaling)
            Field.RenderTransform = fieldScale;

            // Start scan timer
            scanTimer = new System.Windows.Threading.DispatcherTimer();
            scanTimer.Tick += (s, e) => Scan();
            scanTimer.Interval = TimeSpan.FromSeconds(1.0 / scanPerSecond);
            scanTimer.Start();
        }

        /// <summary>
        /// Scan
        /// </summary>
        private void Scan()
        {
            ScanSegments(lidarPos, out List<ScanSegment> scanSegments);
            Draw();

            foreach (ScanSegment seg in scanSegments)
            {
                DrawScan(seg);
            }
        }

        /// <summary>
        /// Draw field and everything
        /// </summary>
        private void Draw()
        {
            Field.Children.Clear();
            DrawBackground();
            DrawField();
        }

        /// <summary>
        /// "Draw" background. Need some object on canvas to get mouse events.
        /// </summary>
        private void DrawBackground()
        {
            // Have 10x10 km rectangle.
            var bg = new Rectangle()
            {
                Fill = Brushes.Silver,
                Width = 10000.0,
                Height = 10000.0f
            };

            Canvas.SetLeft(bg, -5000.0f);
            Canvas.SetTop(bg, -5000.0f);

            Field.Children.Add(bg);
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
                    Stroke = Brushes.Black,
                    StrokeThickness = 0.1f
                };

                Field.Children.Add(line);
            }
        }

        /// <summary>
        /// Draw scan
        /// </summary>
        /// <param name="segment"></param>
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

                Field.Children.Add(line);
            }
        }

        /// <summary>
        /// Scan segments
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="segments"></param>
        private void ScanSegments(Vector2 pos, out List<ScanSegment> segments)
        {
            float scanAngle = (MathF.PI * 2) / numScanPoints;

            ScanSegment scanSegment = new ScanSegment()
            {
                Pose = new Vector3(pos.X, pos.Y, 0),
                IsLast = true
            };

            for (float angle = 0.0f; angle < MathF.PI * 2; angle += scanAngle)
            {
                if (field.RayTrace(pos, angle, maxScanDist, out float hit))
                {
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
                UpdateLidarPosition(e.GetPosition(Field));
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
                UpdateLidarPosition(e.GetPosition(Field));
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
    }
}
