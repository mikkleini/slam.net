using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    /// <summary>
    /// Provides functions related to a log odds of occupancy probability respresentation for cells in a occupancy grid map.
    /// </summary>
    public class GridMapLogOddsFunctions
    {
        private float logOddsOccupied; // The log odds representation of probability used for updating cells as occupied
        private float logOddsFree;     // The log odds representation of probability used for updating cells as free

        /// <summary>
        /// Constructor, sets parameters like free and occupied log odds ratios.
        /// </summary>
        public GridMapLogOddsFunctions()
        {
            SetUpdateFreeFactor(0.4f);
            SetUpdateOccupiedFactor(0.6f);

            /*
            //float probOccupied = 0.6f;
            float probOccupied = 0.9f;
            float oddsOccupied = probOccupied / (1.0f - probOccupied);
            logOddsOccupied = log(oddsOccupied);

            float probFree = 0.4f;
            float oddsFree = probFree / (1.0f - probFree);
            logOddsFree = log(oddsFree);
            */
        }

        /// <summary>
        /// Update cell as occupied
        /// </summary>
        /// <param name="cell">The cell</param>
        public void UpdateSetOccupied(LogOddsCell cell)
        {
            if (cell.Value < 50.0f)
            {
                cell.Value += logOddsOccupied;
            }
        }

        /// <summary>
        /// Update cell as free
        /// </summary>
        /// <param name="cell">The cell</param>
        public void UpdateSetFree(LogOddsCell cell)
        {
            cell.Value += logOddsFree;
        }

        /// <summary>
        /// Update cell as not free
        /// </summary>
        /// <param name="cell">The cell</param>
        public void UpdateUnsetFree(LogOddsCell cell)
        {
            cell.Value -= logOddsFree;
        }

        /**
        * Get the probability value represented by the grid cell.
        * @param cell The cell.
        * @return The probability
        */
        public float GetGridProbability(LogOddsCell cell)
        {
            float odds = MathF.Exp(cell.Value);
            return odds / (odds + 1.0f);

            /*
            float val = cell.logOddsVal;

            //prevent #IND when doing exp(large number).
            if (val > 50.0f) {
              return 1.0f;
            } else {
              float odds = exp(val);
              return odds / (odds + 1.0f);
            }
            */
            //return 0.5f;
        }

        public void SetUpdateFreeFactor(float factor)
        {
            logOddsFree = ProbToLogOdds(factor);
        }

        public void SetUpdateOccupiedFactor(float factor)
        {
            logOddsOccupied = ProbToLogOdds(factor);
        }

        private float ProbToLogOdds(float prob)
        {
            float odds = prob / (1.0f - prob);
            return MathF.Log(odds);
        }
    }
}
