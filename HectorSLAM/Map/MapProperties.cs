using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    /// <summary>
    /// Map properties
    /// </summary>
    public record MapProperties
    {
        /// <summary>
        /// Top-left offset of map in meters
        /// </summary>
        public Vector2 TopLeftOffset { get; internal set; } = new Vector2(-1.0f, -1.0f);

        /// <summary>
        /// Map dimension in pixels
        /// </summary>
        public Point Dimensions { get; internal set; } = new Point(-1, -1);

        /// <summary>
        /// Cell length in meters per pixel
        /// </summary>
        public float CellLength { get; internal set; } = -1.0f;

        /// <summary>
        /// Map limits in meters
        /// </summary>
        public Vector2 Limits => new Vector2(Dimensions.X - 2.0f, Dimensions.Y - 2.0f);

        /// <summary>
        /// Physical size of the map in meters
        /// </summary>
        public Vector2 PhysicalSize => new Vector2(Dimensions.X * CellLength, Dimensions.Y * CellLength);

        /// <summary>
        /// Constructor
        /// </summary>
        public MapProperties()
        {
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="topLeftOffset">Top-left offset</param>
        /// <param name="dimensions">Map dimensions</param>
        /// <param name="cellLength">Cell length</param>
        public MapProperties(Vector2 topLeftOffset, Point dimensions, float cellLength)
        {
            TopLeftOffset = topLeftOffset;
            Dimensions = dimensions;
            CellLength = cellLength;

            // Should set limits also?
        }

        /// <summary>
        /// Has equal dimension ?
        /// </summary>
        /// <param name="other">Other dimensions</param>
        /// <returns>true if equal dimensions</returns>
        public bool HasEqualDimensionProperties(MapProperties other)
        {
            return (Dimensions == other.Dimensions);
        }

        /// <summary>
        /// Has equal transformation properties ?
        /// </summary>
        /// <param name="other">Other dimensions</param>
        /// <returns>true if equal properties</returns>
        public bool HasEqualTransformationProperties(MapProperties other)
        {
            return (TopLeftOffset == other.TopLeftOffset) && (CellLength == other.CellLength);
        }

        /// <summary>
        /// Is point out of map ?
        /// </summary>
        /// <param name="coords">Coordinates</param>
        /// <returns>true if out of map, false if inside map</returns>
        public bool IsPointOutOfMapBounds(Vector2 coords)
        {
            return float.IsNaN(coords.X) || float.IsNaN(coords.Y) ||
                    (coords.X < 0.0f) || (coords.X > Limits.X) || (coords.Y < 0.0f) || (coords.Y > Limits.Y);
        }
    }
}
