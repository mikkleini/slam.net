using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Map
{
    public class OccGridMapBase<ConcreteCellType, ConcreteGridFunctions> : GridMapBase<ConcreteCellType> where ConcreteCellType : ICell
    {
        protected ConcreteGridFunctions concreteGridFunctions;
        protected int currUpdateIndex = 0;
        protected int currMarkOccIndex = -1;
        protected int currMarkFreeIndex = -1;

        public OccGridMapBase(float mapResolution, Point size, Vector2 offset)
            : base(mapResolution, size, offset)
        {
        }

        public void UpdateSetOccupied(int index)
        {
            concreteGridFunctions.UpdateSetOccupied(GetCell(index));
        }

        public void UpdateSetFree(int index)
        {
            concreteGridFunctions.UpdateSetFree(GetCell(index));
        }

        public void UpdateUnsetFree(int index)
        {
            concreteGridFunctions.UpdateUnsetFree(GetCell(index));
        }

        public float GetGridProbabilityMap(int index)
        {
            return concreteGridFunctions.getGridProbability(this->getCell(index));
        }

        bool IsOccupied(int xMap, int yMap)
        {
            return (this->getCell(xMap, yMap).isOccupied());
        }

        bool IsFree(int xMap, int yMap)
        {
            return (this->getCell(xMap, yMap).isFree());
        }

            bool isOccupied(int index)
          {
            return (this->getCell(index).isOccupied());
            }

            bool isFree(int index) const
          {
            return (this->getCell(index).isFree());
            }

            float getObstacleThreshold() const
          {
            ConcreteCellType temp;
            temp.resetGridCell();
            return concreteGridFunctions.getGridProbability(temp);
          }

        void setUpdateFreeFactor(float factor)
        {
            concreteGridFunctions.setUpdateFreeFactor(factor);
        }

        void setUpdateOccupiedFactor(float factor)
        {
            concreteGridFunctions.setUpdateOccupiedFactor(factor);
        }

        /**
         * Updates the map using the given scan data and robot pose
         * @param dataContainer Contains the laser scan data
         * @param robotPoseWorld The 2D robot pose in world coordinates
         */
        void updateByScan(const DataContainer& dataContainer, const Vector3& robotPoseWorld)
        {
            currMarkFreeIndex = currUpdateIndex + 1;
            currMarkOccIndex = currUpdateIndex + 2;

            //Get pose in map coordinates from pose in world coordinates
            Vector3 mapPose(this->getMapCoordsPose(robotPoseWorld));

            //Get a 2D homogenous transform that can be left-multiplied to a robot coordinates vector to get world coordinates of that vector
            Eigen::Affine2f poseTransform((Eigen::Translation2f(
                                                mapPose[0], mapPose[1]) * Eigen::Rotation2Df(mapPose[2])));

            //Get start point of all laser beams in map coordinates (same for alle beams, stored in robot coords in dataContainer)
            Vector2 scanBeginMapf(poseTransform* dataContainer.getOrigo());

            //Get integer vector of laser beams start point
            Point scanBeginMapi(scanBeginMapf[0] +0.5f, scanBeginMapf[1] + 0.5f);

            //Get number of valid beams in current scan
            int numValidElems = dataContainer.getSize();

            //std::cout << "\n maxD: " << maxDist << " num: " << numValidElems << "\n";

            //Iterate over all valid laser beams
            for (int i = 0; i < numValidElems; ++i)
            {

                //Get map coordinates of current beam endpoint
                Vector2 scanEndMapf(poseTransform* (dataContainer.getVecEntry(i)));
            //std::cout << "\ns\n" << scanEndMapf << "\n";

            //add 0.5 to beam endpoint vector for following integer cast (to round, not truncate)
            scanEndMapf.array() += (0.5f);

            //Get integer map coordinates of current beam endpoint
            Point scanEndMapi(scanEndMapf.cast<int>());

            //Update map using a bresenham variant for drawing a line from beam start to beam endpoint in map coordinates
            if (scanBeginMapi != scanEndMapi)
            {
                updateLineBresenhami(scanBeginMapi, scanEndMapi);
            }
        }

            //Tell the map that it has been updated
            this->setUpdated();

            //Increase update index (used for updating grid cells only once per incoming scan)
            currUpdateIndex += 3;
          }

void updateLineBresenhami( const Point& beginMap, const Point& endMap, unsigned int max_length = UINT_MAX)
{

    int x0 = beginMap[0];
    int y0 = beginMap[1];

    //check if beam start point is inside map, cancel update if this is not the case
    if ((x0 < 0) || (x0 >= this->getSizeX()) || (y0 < 0) || (y0 >= this->getSizeY()))
    {
        return;
    }

    int x1 = endMap[0];
    int y1 = endMap[1];

    //std::cout << " x: "<< x1 << " y: " << y1 << " length: " << length << "     ";

    //check if beam end point is inside map, cancel update if this is not the case
    if ((x1 < 0) || (x1 >= this->getSizeX()) || (y1 < 0) || (y1 >= this->getSizeY()))
    {
        return;
    }

    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = util::sign(dx);
    int offset_dy = util::sign(dy) * this->sizeX;

    unsigned int startOffset = beginMap.y() * this->sizeX + beginMap.x();

    //if x is dominant
    if (abs_dx >= abs_dy)
    {
        int error_y = abs_dx / 2;
        bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, startOffset);
    }
    else
    {
        //otherwise y is dominant
        int error_x = abs_dy / 2;
        bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, startOffset);
    }

    unsigned int endOffset = endMap.y() * this->sizeX + endMap.x();
    this->bresenhamCellOcc(endOffset);

}

        private void BresenhamCellFree(int offset)
        {
            ConcreteCellType & cell(this->getCell(offset));

            if (cell.updateIndex < currMarkFreeIndex)
            {
                concreteGridFunctions.updateSetFree(cell);
                cell.updateIndex = currMarkFreeIndex;
            }
        }

        private void BresenhamCellOcc(int offset)
        {
            ConcreteCellType & cell(this->getCell(offset));

            if (cell.updateIndex < currMarkOccIndex)
            {

                //if this cell has been updated as free in the current iteration, revert this
                if (cell.updateIndex == currMarkFreeIndex)
                {
                    concreteGridFunctions.updateUnsetFree(cell);
                }

                concreteGridFunctions.updateSetOccupied(cell);
                //std::cout << " setOcc " << "\n";
                cell.updateIndex = currMarkOccIndex;
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
