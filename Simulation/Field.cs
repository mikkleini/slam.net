using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using Box2D.NetStandard;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;

namespace Simulation
{
    /// <summary>
    /// Simulated field
    /// </summary>
    public class Field
    {
        private World world;

        /// <summary>
        /// Constructor
        /// </summary>
        public Field()
        {
            world = new World(new Vector2(0, 0));
        }

        /// <summary>
        /// Ray-tracing standard deviation error in meters.
        /// </summary>
        public float RayTraceError { get; set; } = 0.01f;

        /// <summary>
        /// Create default field
        /// </summary>
        public void CreateDefaultField()
        {
            Vector2[] outerEdges = new Vector2[]
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

            AddEdges(outerEdges, true);

            Vector2[] innerEdges = new Vector2[]
            {
                new Vector2(-10.0f, -5.0f),
                new Vector2(-8.0f,  -5.0f),
                new Vector2(-6.0f,  +1.0f),
                new Vector2(-8.0f, +2.0f),
            };

            AddEdges(innerEdges, true);
        }

        /// <summary>
        /// Add edges to the field
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="closeLoop">Close loop ?</param>
        public void AddEdges(Vector2[] vertices, bool closeLoop)
        {
            var bodyDef = new BodyDef
            {
                type = BodyType.Static,
                position = new Vector2(0, 0)
            };

            var body = world.CreateBody(bodyDef);

            // Create edge shapes. They support two-side collision
            Vector2 prev = vertices.First();
            foreach (Vector2 next in vertices.Skip(1))
            {
                var edgeShape = new EdgeShape(prev, next);
                var fixtureDef = new FixtureDef
                {
                    shape = edgeShape
                };

                body.CreateFixture(fixtureDef);
                prev = next;
            }

            // Close loop ?
            if (closeLoop)
            {
                Vector2 next = vertices.First();

                var edgeShape = new EdgeShape(prev, next);
                var fixtureDef = new FixtureDef
                {
                    shape = edgeShape
                };

                body.CreateFixture(fixtureDef);
            }
        }

        /// <summary>
        /// Get all edges as lines (two points)
        /// </summary>
        /// <returns></returns>
        public IEnumerable<(Vector2, Vector2)> GetEdges()
        {
            // Iterate over all bodies
            Body body = world.GetBodyList();
            while (body != null)
            {
                // Iterate over all fixtures
                Fixture fixture = body.GetFixtureList();
                while (fixture != null)
                {
                    switch (fixture.Shape)
                    {
                        case EdgeShape edgeShape:
                            yield return new(edgeShape.Vertex1, edgeShape.Vertex2);
                            break;

                        case ChainShape chainShape:
                            //RenderPolygon(chainShape.Vertices, Brushes.Black, 0.1);
                            break;

                        case PolygonShape polyShape:
                            //RenderPolygon(polyShape.GetVertices(), Brushes.Black, 0.1);
                            break;
                    }

                    fixture = fixture.GetNext();
                }

                body = body.GetNext();
            }
        }

        /// <summary>
        /// Ray-trace and get closest hit
        /// </summary>
        /// <param name="pos">Ray emitting position</param>
        /// <param name="angle">Ray emitting angle</param>
        /// <param name="maxDist">Maximum measurement distance</param>
        /// <param name="hit">Closest hit distance (if hit)</param>
        /// <returns>trye if hit, false if not</returns>
        public bool RayTrace(Vector2 pos, float angle, float maxDist, out float hit)
        {
            Vector2 endPos = pos + new Vector2(maxDist * MathF.Cos(angle), maxDist * MathF.Sin(angle));
            float radius = float.MaxValue;

            // Ray-cast and find closest hit
            world.RayCast((fixt, hitPos, normal, fract) =>
            {
                radius = Math.Min(radius, fract * maxDist);
            }, pos, endPos);

            // Got any hit ?
            if (radius < float.MaxValue)
            {
                hit = radius;
                return true;
            }

            hit = 0.0f;
            return false;
        }
    }
}
