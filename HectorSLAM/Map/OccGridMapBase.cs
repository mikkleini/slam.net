﻿using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BaseSLAM;
using HectorSLAM.Util;

namespace HectorSLAM.Map
{
    public class OccGridMapBase : GridMapBase
    {
        /// <summary>
        /// Array used for caching data.
        /// </summary>
        private readonly CachedMapElement[] array = null;

        // Variables
        private int currCacheIndex = 0;
        private int currUpdateIndex = 0;
        private int currMarkOccIndex = -1;
        private int currMarkFreeIndex = -1;

        private float oddsOccupied = 0.9f;
        private float oddsFree = 0.4f;
        private float logOddsOccupied; // The log odds representation of probability used for updating cells as occupied
        private float logOddsFree;     // The log odds representation of probability used for updating cells as free

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="mapResolution">Map resolution in meters per pixel</param>
        /// <param name="size">Map size in pixels</param>
        /// <param name="offset">Offset if meters</param>
        public OccGridMapBase(float mapResolution, Point size, Vector2 offset)
            : base(mapResolution, size, offset)
        {
            // Create cache array
            int length = size.X * size.Y;
            array = new CachedMapElement[length];
            for (int i = 0; i < length; ++i)
            {
                array[i].index = -1;
            }

            // Set logs
            logOddsFree = ProbToLogOdds(oddsFree);
            logOddsOccupied = ProbToLogOdds(oddsOccupied);
        }

        /// <summary>
        /// Cell "free" update factor
        /// </summary>
        public float UpdateFreeFactor
        {
            get => oddsFree;
            set
            {
                oddsFree = value;
                logOddsFree = ProbToLogOdds(oddsFree);
            }
        }

        /// <summary>
        /// Cell "occupied" update factor
        /// </summary>
        public float UpdateOccupiedFactor
        {
            get => oddsOccupied;
            set
            {
                oddsOccupied = value;
                logOddsOccupied = ProbToLogOdds(oddsOccupied);
            }
        }

        /// <summary>
        /// Probability to logarithmic odds
        /// </summary>
        /// <param name="prob">Probability</param>
        /// <returns>Logarithmic value</returns>
        private static float ProbToLogOdds(float prob)
        {
            float odds = prob / (1.0f - prob);
            return MathF.Log(odds);
        }

        /// <summary>
        /// Get cached probability of the cell
        /// </summary>
        /// <param name="index">Cell index</param>
        /// <returns>Probability </returns>
        public float GetCachedProbability(int index)
        {
            if (array[index].index != currCacheIndex)
            {
                float odds = MathF.Exp(mapArray[index].Value);
                array[index].value = odds / (odds + 1.0f);
                array[index].index = currCacheIndex;
            }

            return array[index].value;
        }

        /// <summary>
        /// Updates the map using the given scan data and robot pose
        /// </summary>
        /// <param name="scan">Scan points</param>
        /// <param name="robotPoseWorld">Robot world pose (X and Y in meters, Z in radians)</param>
        public void UpdateByScan(ScanCloud scan, Vector3 robotPoseWorld)
        {
            currMarkFreeIndex = currUpdateIndex + 1;
            currMarkOccIndex = currUpdateIndex + 2;

            // Get a 2D homogenous transform that can be left-multiplied to a robot coordinates vector to get world coordinates of that vector
            Matrix3x2 poseTransform =
                Matrix3x2.CreateTranslation(robotPoseWorld.X, robotPoseWorld.Y) *  // Translate position
                Matrix3x2.CreateRotation(robotPoseWorld.Z) *                       // Rotate                
                Matrix3x2.CreateScale(Properties.ScaleToMap);                      // Meters to pixels

            // Get start point of all laser beams
            Vector2 scanBeginMapf = Vector2.Transform(scan.Pose.ToVector2(), poseTransform);
            Point scanBeginMapi = scanBeginMapf.ToRoundPoint();

            //Iterate over all valid laser beams
            foreach (Vector2 scanPoint in scan.Points)
            {
                // Get map coordinates of the beam endpoint
                Vector2 scanEndMapf = Vector2.Transform(scanPoint, poseTransform);
                Point scanEndMapi = scanEndMapf.ToRoundPoint();

                // Update map using a bresenham variant for drawing a line from beam start to beam endpoint in map coordinates
                if (scanBeginMapi != scanEndMapi)
                {
                    UpdateLineBresenhami(scanBeginMapi, scanEndMapi);
                }
            }

            // Increase update index (used for updating grid cells only once per incoming scan)
            currUpdateIndex += 3;

            // Invalidate previous cache
            currCacheIndex++;
        }

        /// <summary>
        /// Draw beam between beginning and end point
        /// </summary>
        /// <param name="begin"></param>
        /// <param name="end"></param>
        private void UpdateLineBresenhami(Point begin, Point end)
        {
            // Check if beam start and end point is inside map
            if (!Properties.IsPointInDimensions(begin) || !Properties.IsPointInDimensions(end))
            {
                return;
            }

            int dx = end.X - begin.X;
            int dy = end.Y - begin.Y;

            int abs_dx = Math.Abs(dx);
            int abs_dy = Math.Abs(dy);

            int offset_dx = Math.Sign(dx);
            int offset_dy = Math.Sign(dy) * Dimensions.X;

            int startOffset = begin.Y * Dimensions.X + begin.X;

            //if x is dominant
            if (abs_dx >= abs_dy)
            {
                int error_y = abs_dx / 2;
                Bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, startOffset);
            }
            else
            {
                //otherwise y is dominant
                int error_x = abs_dy / 2;
                Bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, startOffset);
            }

            int endOffset = end.Y * Dimensions.X + end.X;

            BresenhamCellOcc(endOffset);
        }

        private void BresenhamCellFree(int index)
        {
            if (mapArray[index].UpdateIndex < currMarkFreeIndex)
            {
                mapArray[index].Value += logOddsFree;
                mapArray[index].UpdateIndex = currMarkFreeIndex;
            }
        }

        private void BresenhamCellOcc(int index)
        {
            if (mapArray[index].UpdateIndex < currMarkOccIndex)
            {
                // if this cell has been updated as free in the current iteration, revert this
                if (mapArray[index].UpdateIndex == currMarkFreeIndex)
                {
                    mapArray[index].Value -= logOddsFree;
                }

                if (mapArray[index].Value < 50.0f)
                {
                    mapArray[index].Value += logOddsOccupied;
                }

                mapArray[index].UpdateIndex = currMarkOccIndex;
            }
        }

        private void Bresenham2D(int abs_da, int abs_db, int error_b, int offset_a, int offset_b, int offset)
        {
            BresenhamCellFree(offset);

            int end = abs_da - 1;

            for (int i = 0; i < end; ++i)
            {
                offset += offset_a;
                error_b += abs_db;

                if (error_b >= abs_da)
                {
                    offset += offset_b;
                    error_b -= abs_da;
                }

                BresenhamCellFree(offset);
            }
        }
    }
}
