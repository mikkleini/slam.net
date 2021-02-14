using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
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
using Box2D.NetStandard;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;
using BaseSLAM;

namespace Simulation
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private World world;
        private PolygonShape outerShape;
        private Body outerBody;
        private Body innerBody;
        private const int numScanPoints = 100;
        private const float scanAngle = (MathF.PI * 2) / numScanPoints;
        private const float maxScanDist = 40.0f; // Meters

        public MainWindow()
        {
            InitializeComponent();

            world = new World(new Vector2(0, 0));

            //Field.RenderTransform = new TranslateTransform(250, 250);
            Field.RenderTransform = new ScaleTransform(20, 20);

            CreateField();
            RenderWorld();
        }

        private void CreateField()
        {
            BodyDef outerBodyDef = new BodyDef
            {
                type = BodyType.Static,
                position = new Vector2(0, 0)
            };

            outerBody = world.CreateBody(outerBodyDef);

            Vector2[] edges = new Vector2[]
            {
                new Vector2(-15.0f, -10.0f),
                new Vector2(+15.0f, -10.0f),
                new Vector2(+15.0f,  -7.0f),
                new Vector2(+12.0f,  -6.0f),
                new Vector2(+10.0f,  -2.0f),
                new Vector2(+15.0f,  -2.0f),
                new Vector2(+15.0f, +10.0f),
                new Vector2(-7.0f,  +10.0f),
                new Vector2(-7.0f,  +5.0f),
                new Vector2(-8.0f,  +5.0f),
                new Vector2(-8.0f,  +10.0f),
                new Vector2(-15.0f, +10.0f),
            };

            //outerShape = new PolygonShape(10, 200);
            var cs = new ChainShape();
            cs.CreateLoop(edges);


            FixtureDef outerDef = new FixtureDef
            {
                shape = cs
            };

            outerBody.CreateFixture(outerDef);



            // -----------------------------

            BodyDef innerBodyDef = new BodyDef
            {
                type = BodyType.Static,
                position = new Vector2(0, 0)
            };

            innerBody = world.CreateBody(innerBodyDef);

            Vector2[] innerEdges = new Vector2[]
            {
                new Vector2(-10.0f, -5.0f),
                new Vector2(-8.0f,  -5.0f),
                new Vector2(-6.0f,  +1.0f),
                new Vector2(-8.0f, +2.0f),
            };

            var innerShape = new PolygonShape(innerEdges);

            //outerShape = new PolygonShape(edges);


            FixtureDef innerDef = new FixtureDef
            {
                shape = innerShape
            };

            innerBody.CreateFixture(innerDef);
        }

        private void RenderWorld()
        {
            //RenderPolygon(outerShape.GetVertices(), Brushes.Black);

            Body b = world.GetBodyList();

            while (b != null)
            {

                //world.GetBodyList().GetNext

                //outerBody.GetFixtureList().GetNext();

                Fixture f = b.GetFixtureList();



                while (f != null)
                {
                    switch (f.Shape)
                    {
                        case ChainShape chainShape:
                            RenderPolygon(chainShape.Vertices, Brushes.Black, 0.1);
                            break;

                        case PolygonShape polyShape:
                            RenderPolygon(polyShape.GetVertices(), Brushes.Black, 0.1);
                            break;
                    }

                    f = f.GetNext();
                }

                b = b.GetNext();
            }

            
        }

        /// <summary>
        /// Render polygon
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="brush"></param>
        private void RenderPolygon(IEnumerable<Vector2> vertices, Brush brush, double thickness)
        {
            Polygon poly = new Polygon()
            {
                Fill = Brushes.Silver
            };

            //Field.Children.Add(poly);

            Vector2 prev = vertices.Last();

            foreach (Vector2 v in vertices)
            {
                Line line = new Line()
                {
                    X1 = prev.X,
                    Y1 = prev.Y,
                    X2 = v.X,
                    Y2 = v.Y,
                    Stroke = brush,
                    StrokeThickness = thickness
                };

                //poly.Points.Add(new Point(v.X, v.Y));

                Field.Children.Add(line);
                prev = v;
            }
        }

        private void Scan(Vector2 pos, out List<ScanSegment> segments)
        {
            ScanSegment scanSegment = new ScanSegment()
            {
                Pose = new Vector3(pos.X, pos.Y, 0),
                IsLast = true
            };

            for (float angle = 0.0f; angle < MathF.PI * 2; angle += scanAngle)
            {
                Vector2 endPos = pos + new Vector2(maxScanDist * MathF.Cos(angle), maxScanDist * MathF.Sin(angle));
                List<float> hits = new List<float>();

                world.RayCast((fixt, hitPos, normal, fract) =>
                {
                    hits.Add((hitPos - pos).Length());
                }, pos, endPos);

                if (hits.Count > 0)
                {
                    hits.Sort();
                    scanSegment.Rays.Add(new Ray(angle, hits[0]));
                }
            }

            segments = new List<ScanSegment>()
            {
                scanSegment
            };
        }

        private void Field_MouseMove(object sender, MouseEventArgs e)
        {

            //if (e.LeftButton == MouseButtonState.Pressed)
            {

                Point p = e.GetPosition(Field);
                Vector2 pos = new Vector2((float)p.X, (float)p.Y);

                Scan(pos, out List<ScanSegment> scanSegments);

                Field.Children.Clear();
                
                var bg = new Rectangle()
                {
                    Fill = Brushes.Silver,
                    Width = 200.0f,
                    Height = 200.0f
                };

                Canvas.SetLeft(bg, -100.0f);
                Canvas.SetTop(bg, -100.0f);

                Field.Children.Add(bg);

                RenderWorld();

                this.Title = $"{p.X} x {p.Y}";

                foreach (ScanSegment seg in scanSegments)
                {
                    RenderScan(seg);
                }
            }
        }

        private void RenderScan(ScanSegment segment)
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
    }
}
