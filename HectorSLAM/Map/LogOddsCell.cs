using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    /// <summary>
    /// Provides a log odds of occupancy probability representation for cells in a occupancy grid map.
    /// </summary>
    public class LogOddsCell : ICell
    {
        public float logOddsVal; // The log odds representation of occupancy probability.
        public int updateIndex;

        /// <summary>
        /// Constructor
        /// </summary>
        public LogOddsCell()
        {

        }

        /// <summary>
        /// Sets the cell value.
        /// </summary>
        /// <param name="value">Value</param>
        public void Set(float value)
        {
            logOddsVal = value;
        }

        /// <summary>
        /// Returns the value of the cell.
        /// </summary>
        /// <returns>Value</returns>
        public float GetValue()
        {
            return logOddsVal;
        }

        /// <summary>
        /// Returns whether the cell is occupied.
        /// </summary>
        /// <returns>true if occupied</returns>
        public bool IsOccupied()
        {
            return logOddsVal > 0.0f;
        }

        /// <summary>
        /// Returns whether the cell is free.
        /// </summary>
        /// <returns>true if free</returns>
        public bool IsFree()
        {
            return logOddsVal < 0.0f;
        }

        /// <summary>
        /// Reset cell to prior probability.
        /// </summary>
        public void ResetGridCell()
        {
            logOddsVal = 0.0f;
            updateIndex = -1;
        }
    }
}
