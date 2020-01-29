using HectorSLAM.Scan;
using HectorSLAM.Util;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    public class OccGridMapBase<T> : GridMapBase<T> where T : LogOddsCell
    {
        protected GridMapLogOddsFunctions gridFunctions;
        protected int currUpdateIndex = 0;
        protected int currMarkOccIndex = -1;
        protected int currMarkFreeIndex = -1;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="mapResolution">Map resolution</param>
        /// <param name="size">Map size</param>
        /// <param name="offset">Offset</param>
        public OccGridMapBase(float mapResolution, Point size, Vector2 offset)
            : base(mapResolution, size, offset)
        {
            gridFunctions = new GridMapLogOddsFunctions();
        }

        public void UpdateSetOccupied(int index)
        {
            gridFunctions.UpdateSetOccupied(GetCell(index));
        }

        public void UpdateSetFree(int index)
        {
            gridFunctions.UpdateSetFree(GetCell(index));
        }

        public void UpdateUnsetFree(int index)
        {
            gridFunctions.UpdateUnsetFree(GetCell(index));
        }

        public float GetGridProbabilityMap(int index)
        {
            return gridFunctions.GetGridProbability(GetCell(index));
        }

        public void SetUpdateFreeFactor(float factor)
        {
            gridFunctions.SetUpdateFreeFactor(factor);
        }

        public void SetUpdateOccupiedFactor(float factor)
        {
            gridFunctions.SetUpdateOccupiedFactor(factor);
        }

        public bool IsOccupied(int xMap, int yMap)
        {
            return GetCell(xMap, yMap).IsOccupied;
        }

        public bool IsFree(int xMap, int yMap)
        {
            return GetCell(xMap, yMap).IsFree;
        }

        public bool IsOccupied(int index)
        {
            return GetCell(index).IsOccupied;
        }

        public bool IsFree(int index)
        {
            return GetCell(index).IsFree;
        }

        public float GetObstacleThreshold()
        {
            LogOddsCell temp = new LogOddsCell();
            temp.Reset();
            return gridFunctions.GetGridProbability(temp);
        }

        /**
         * Updates the map using the given scan data and robot pose
         * @param dataContainer Contains the laser scan data
         * @param robotPoseWorld The 2D robot pose in world coordinates
         */
        public void UpdateByScan(DataContainer dataContainer, Vector3 robotPoseWorld)
        {
            currMarkFreeIndex = currUpdateIndex + 1;
            currMarkOccIndex = currUpdateIndex + 2;

            // Get pose in map coordinates from pose in world coordinates
            Vector3 mapPose = GetMapCoordsPose(robotPoseWorld);

            // Get a 2D homogenous transform that can be left-multiplied to a robot coordinates vector to get world coordinates of that vector
            Matrix4x4 poseTransform = Matrix4x4.CreateTranslation(mapPose.X, mapPose.Y, 0) * Matrix4x4.CreateRotationZ(mapPose.Z);
            //Eigen::Affine2f poseTransform((Eigen::Translation2f(mapPose[0], mapPose[1]) * Eigen::Rotation2Df(mapPose[2])));

            // Get start point of all laser beams in map coordinates (same for alle beams, stored in robot coords in dataContainer)
            Vector2 scanBeginMapf = Vector2.Transform(dataContainer.Origo, poseTransform);

            // Get integer vector of laser beams start point
            Point scanBeginMapi = scanBeginMapf.ToRoundPoint();

            //std::cout << "\n maxD: " << maxDist << " num: " << numValidElems << "\n";

            //Iterate over all valid laser beams
            for (int i = 0; i < dataContainer.Count; ++i)
            {
                //Get map coordinates of current beam endpoint
                Vector2 scanEndMapf = Vector2.Transform(dataContainer[i], poseTransform);
                //std::cout << "\ns\n" << scanEndMapf << "\n";

                //Get integer map coordinates of current beam endpoint
                Point scanEndMapi = scanEndMapf.ToRoundPoint();

                //Update map using a bresenham variant for drawing a line from beam start to beam endpoint in map coordinates
                if (scanBeginMapi != scanEndMapi)
                {
                    UpdateLineBresenhami(scanBeginMapi, scanEndMapi);
                }
            }

            //Tell the map that it has been updated
            SetUpdated();

            //Increase update index (used for updating grid cells only once per incoming scan)
            currUpdateIndex += 3;
        }

        private void UpdateLineBresenhami(Point beginMap, Point endMap, uint max_length = uint.MaxValue)
        {
            // Check if beam start point is inside map, cancel update if this is not the case
            if (!HasGridValue(beginMap.X, beginMap.Y))
            {
                return;
            }

            // Check if beam end point is inside map, cancel update if this is not the case
            if (!HasGridValue(endMap.X, endMap.Y))
            {
                return;
            }

            int dx = endMap.X - beginMap.X;
            int dy = endMap.Y - beginMap.Y;

            int abs_dx = Math.Abs(dx);
            int abs_dy = Math.Abs(dy);

            int offset_dx = Math.Sign(dx);
            int offset_dy = Math.Sign(dy) * Dimensions.X;

            int startOffset = beginMap.Y * Dimensions.X + beginMap.X;

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

            int endOffset = endMap.Y * Dimensions.X + endMap.X;

            BresenhamCellOcc(endOffset);
        }

        private void BresenhamCellFree(int offset)
        {
            LogOddsCell cell = GetCell(offset);

            if (cell.UpdateIndex < currMarkFreeIndex)
            {
                gridFunctions.UpdateSetFree(cell);
                cell.UpdateIndex = currMarkFreeIndex;
            }
        }

        private void BresenhamCellOcc(int offset)
        {
            T cell = GetCell(offset);

            if (cell.UpdateIndex < currMarkOccIndex)
            {

                //if this cell has been updated as free in the current iteration, revert this
                if (cell.UpdateIndex == currMarkFreeIndex)
                {
                    gridFunctions.UpdateUnsetFree(cell);
                }

                gridFunctions.UpdateSetOccupied(cell);
                //std::cout << " setOcc " << "\n";
                cell.UpdateIndex = currMarkOccIndex;
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
