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
        /// Physical size of the map in meters
        /// </summary>
        public Vector2 PhysicalSize { get; }

        /// <summary>
        /// Map dimensions in pixels
        /// </summary>
        public Point Dimensions { get; init; }

        /// <summary>
        /// Cell length in meters per pixel
        /// </summary>
        public float CellLength { get; init; }

        /// <summary>
        /// Scale of the map in pixels per meter (inverse of CellLength)
        /// </summary>
        public float ScaleToMap => 1.0f / CellLength;

        /// <summary>
        /// Top-left offset of map in meters
        /// </summary>
        public Vector2 Offset { get; init; }

        /// <summary>
        /// Map limits in meters
        /// </summary>
        public Vector2 Limits => new Vector2(Dimensions.X - 2.0f, Dimensions.Y - 2.0f); // TODO Where does the -2 come from ?

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="cellLength">Cell length in meters per pixel</param>
        /// <param name="dimensions">Map dimensions in pixels</param>
        /// <param name="offset">Top-left offset of map in meters</param>
        public MapProperties(float cellLength, Point dimensions, Vector2 offset)
        {
            CellLength = cellLength;
            Dimensions = dimensions;
            Offset = offset;
            PhysicalSize = new Vector2(Dimensions.X * CellLength, Dimensions.Y * CellLength);
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
            return (Offset == other.Offset) && (CellLength == other.CellLength);
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

        /// <summary>
        /// Is point within dimensions
        /// </summary>
        /// <param name="p">Point</param>
        /// <returns>true if point is within dimensions, false if not</returns>
        public bool IsPointInDimensions(Point p)
        {
            return (p.X >= 0) && (p.Y >= 0) && (p.X < Dimensions.X) && (p.Y < Dimensions.Y);
        }
    }
}
