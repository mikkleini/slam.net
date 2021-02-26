using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace HectorSLAM.Map
{
    /// <summary>
    /// Provides a log odds of occupancy probability representation for cells in a occupancy grid map.
    /// </summary>
    public struct LogOddsCell
    {
        /// <summary>
        /// Update index
        /// </summary>
        public int UpdateIndex;

        /// <summary>
        /// The log odds representation of occupancy probability.
        /// </summary>
        public float Value;

        /// <summary>
        /// Returns whether the cell is occupied.
        /// </summary>
        /// <returns>true if occupied</returns>
        public bool IsOccupied => Value > 0.0f;

        /// <summary>
        /// Returns whether the cell is free.
        /// </summary>
        /// <returns>true if free</returns>
        public bool IsFree => Value < 0.0f;

        /// <summary>
        /// Reset cell to prior probability.
        /// </summary>
        public void Reset()
        {
            Value = 0.0f;
            UpdateIndex = -1;
        }

        /// <summary>
        /// Cell as string (for debuggin)
        /// </summary>
        /// <returns>Text</returns>
        public override string ToString()
        {
            return $"{Value:f3}, {UpdateIndex})";
        }
    }
}
