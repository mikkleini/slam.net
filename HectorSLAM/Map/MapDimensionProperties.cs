using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    /// <summary>
    /// Map dimension properties
    /// </summary>
    public class MapDimensionProperties
    {
        /// <summary>
        /// Top-left offset
        /// </summary>
        public Vector2 TopLeftOffset { get; internal set; } = new Vector2(-1.0f, -1.0f);

        /// <summary>
        /// Map dimensions
        /// </summary>
        public Point Dimensions { get; internal set; } = new Point(-1, -1);

        /// <summary>
        /// Cell length
        /// </summary>
        public float CellLength { get; internal set; } = -1.0f;

        /// <summary>
        /// Map limits
        /// </summary>
        public Vector2 MapLimitsF => new Vector2(Dimensions.X - 2.0f, Dimensions.Y - 2.0f);
        
        /// <summary>
        /// Constructor
        /// </summary>
        public MapDimensionProperties()
        {
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="topLeftOffset">Top-left offset</param>
        /// <param name="dimensions">Map dimensions</param>
        /// <param name="cellLength">Cell length</param>
        public MapDimensionProperties(Vector2 topLeftOffset, Point dimensions, float cellLength)
        {
            TopLeftOffset = topLeftOffset;
            Dimensions = dimensions;
            CellLength = cellLength;
        }

        /// <summary>
        /// Comparison
        /// </summary>
        /// <param name="a">A</param>
        /// <param name="b">B</param>
        /// <returns>true if equal</returns>
        public static bool operator ==(MapDimensionProperties a, MapDimensionProperties b)
        {
            return (a.TopLeftOffset == b.TopLeftOffset) && (a.Dimensions == b.Dimensions) && (a.CellLength == b.CellLength);
        }

        /// <summary>
        /// Negative comparison
        /// </summary>
        /// <param name="a">A</param>
        /// <param name="b">B</param>
        /// <returns>true if equal</returns>
        public static bool operator !=(MapDimensionProperties a, MapDimensionProperties b)
        {
            return (a.TopLeftOffset != b.TopLeftOffset) || (a.Dimensions != b.Dimensions) || (a.CellLength != b.CellLength);
        }

        /// <summary>
        /// Has equal dimension ?
        /// </summary>
        /// <param name="other">Other dimensions</param>
        /// <returns>true if equal dimensions</returns>
        public bool HasEqualDimensionProperties(MapDimensionProperties other)
        {
            return (Dimensions == other.Dimensions);
        }

        /// <summary>
        /// Has equal transformation properties ?
        /// </summary>
        /// <param name="other">Other dimensions</param>
        /// <returns>true if equal properties</returns>
        public bool HasEqualTransformationProperties(MapDimensionProperties other)
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
            return (coords.X < 0.0f) || (coords.X > MapLimitsF.X) || (coords.Y < 0.0f) || (coords.Y > MapLimitsF.Y);
        }

        /// <summary>
        /// Equals to other ?
        /// </summary>
        /// <param name="other">Other object</param>
        /// <returns>true if equals to other object</returns>
        public override bool Equals(object other)
        {
            if (other is MapDimensionProperties otherDim)
            {
                return this == otherDim;
            }

            return false;
        }

        /// <summary>
        /// Get hash code
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            return TopLeftOffset.GetHashCode() + 1000 * Dimensions.GetHashCode() + 4444 * CellLength.GetHashCode();
        }
    }
}
