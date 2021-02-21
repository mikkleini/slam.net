using System;
using System.Collections.Generic;
using System.Text;
using System.Numerics;
using System.Drawing;
using BaseSLAM;
using HectorSLAM.Util;
using System.Runtime.CompilerServices;

namespace HectorSLAM.Map
{
    public class GridMapBase<T> where T : ICell
    {
        private int lastUpdateIndex = -1;

        ///< Map representation used with plain pointer array.
        protected T[] mapArray = null;

        // Scaling factor from world to map.
        public float ScaleToMap { get; private set; }

        protected Matrix4x4 worldTmap;     ///< Homogenous transform from map to world coordinates.
        protected Matrix4x4 mapTworld;     ///< Homogenous transform from world to map coordinates.

        /// <summary>
        /// Map properties
        /// </summary>
        public MapProperties Properties { get; } = new MapProperties();

        public Point Dimensions => Properties.Dimensions;

        /// <summary>
        /// Constructor, creates grid representation and transformations.
        /// </summary>
        /// <param name="mapResolution">Map resolution in meters per pixel</param>
        /// <param name="size">Map size in pixels</param>
        /// <param name="offset">Offset if meters</param>
        public GridMapBase(float mapResolution, Point size, Vector2 offset)
        {
            SetMapGridSize(size);
            SetMapTransformation(offset, mapResolution);
            Clear();
        }

        /**
        * Indicates if given x and y are within map bounds
        * @return True if coordinates are within map bounds
        */
        public bool HasGridValue(int x, int y)
        {
            return (x >= 0) && (y >= 0) && (x < Dimensions.X) && (y < Dimensions.Y);
        }


        public void Reset()
        {
            Clear();
        }

        /// <summary>
        /// Resets the grid cell values by using the resetGridCell() function.
        /// </summary>
        public void Clear()
        {
            int size = Dimensions.X * Dimensions.Y;

            for (int i = 0; i < size; ++i)
            {
                mapArray[i].Reset();
            }

            //mapArray[0].set(1.0f);
            //mapArray[size-1].set(1.0f);
        }

        /**
         * Allocates memory for the two dimensional pointer array for map representation.
         */
        public void AllocateArray(Point newMapDims)
        {
            mapArray = new T[newMapDims.X * newMapDims.Y];
            Properties.Dimensions = newMapDims;


            int length = Dimensions.X * Dimensions.Y;

            for (int i = 0; i < length; ++i)
            {
                mapArray[i] = (T)Activator.CreateInstance(typeof(T));
            }

        }

        public void DeleteArray()
        {
            if (mapArray != null)
            {
                mapArray = null;
                Properties.Dimensions = new Point(-1, -1);
            }
        }

        /// <summary>
        /// Get cell by coordinates
        /// </summary>
        /// <param name="x">X</param>
        /// <param name="y">Y</param>
        /// <returns>Cell</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T GetCell(int x, int y)
        {
            return mapArray[y * Dimensions.X + x];
        }

        /// <summary>
        /// Get cell by index
        /// </summary>
        /// <param name="index">Arry index</param>
        /// <returns>Cell</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T GetCell(int index)
        {
            return mapArray[index];
        }

        public void SetMapGridSize(Point newMapDims)
        {
            if (newMapDims != Dimensions)
            {
                DeleteArray();
                AllocateArray(newMapDims);
                Reset();
            }
        }

        /// <summary>
        /// Copy Constructor, only needed if pointer members are present.
        /// </summary>
        /// <param name="other"></param>
        public GridMapBase(GridMapBase<T> other)
        {
            // TODO:
            AllocateArray(other.Dimensions);
            //*this = other;
        }

        /**
         * Assignment operator, only needed if pointer members are present.
         */
        /*
        public static GridMapBase operator=(GridMapBase other)
        {
            if (!(mapDimensionProperties == other.mapDimensionProperties))
            {
                setMapGridSize(other.mapDimensionProperties.getMapDimensions());
            }

            mapDimensionProperties = other.mapDimensionProperties;

            worldTmap = other.worldTmap;
            mapTworld = other.mapTworld;
            worldTmap3D = other.worldTmap3D;

            scaleToMap = other.scaleToMap;

            //@todo potential resize
            int sizeX = getSizeX();
            int sizeY = getSizeY();

            size_t concreteCellSize = sizeof(ConcreteCellType);

            memcpy(mapArray, other.mapArray, sizeX * sizeY * concreteCellSize);

            return *this;
        }
        */

        /**
         * Returns the world coordinates for the given map coords.
         */
        public Vector2 GetWorldCoords(Vector2 mapCoords)
        {
            return Vector2.Transform(mapCoords, worldTmap);
        }

        /**
         * Returns the map coordinates for the given world coords.
         */
        public Vector2 GetMapCoords(Vector2 worldCoords)
        {
            return Vector2.Transform(worldCoords, mapTworld);
        }

        /**
         * Returns the world pose for the given map pose.
         */
        public Vector3 GetWorldCoordsPose(Vector3 mapPose)
        {
            Vector2 worldCoords = Vector2.Transform(mapPose.ToVector2(), worldTmap);
            return new Vector3(worldCoords.X, worldCoords.Y, mapPose.Z);
        }

        /**
        * Returns the map pose for the given world pose.
        */
        public Vector3 GetMapCoordsPose(Vector3 worldPose)
        {
            Vector2 mapCoords = Vector2.Transform(worldPose.ToVector2(), mapTworld);
            return new Vector3(mapCoords.X, mapCoords.Y, worldPose.Z);
        }

        private void SetDimensionProperties(MapProperties newMapDimProps)
        {
            //Grid map cell number has changed
            if (!newMapDimProps.HasEqualDimensionProperties(Properties))
            {
                SetMapGridSize(newMapDimProps.Dimensions);
            }

            //Grid map transformation/cell size has changed
            if (!newMapDimProps.HasEqualTransformationProperties(Properties))
            {
                SetMapTransformation(newMapDimProps.TopLeftOffset, newMapDimProps.CellLength);
            }
        }

        /// <summary>
        /// Set the map transformations
        /// </summary>
        /// <param name="topLeftOffset">The origin of the map coordinate system in world coordinates (meters)</param>
        /// <param name="cellLength">The cell length of the grid map</param>
        private void SetMapTransformation(Vector2 topLeftOffset, float cellLength)
        {
            Properties.CellLength = cellLength;
            Properties.TopLeftOffset = topLeftOffset;

            ScaleToMap = 1.0f / cellLength;

            mapTworld = Matrix4x4.CreateScale(ScaleToMap) * Matrix4x4.CreateTranslation(topLeftOffset.X, topLeftOffset.Y, 0.0f);
            //mapTworld = Eigen::AlignedScaling2f(ScaleToMap, ScaleToMap) * Eigen::Translation2f(topLeftOffset.X, topLeftOffset.Y);

            if (!Matrix4x4.Invert(mapTworld, out worldTmap))
            {
                throw new Exception("Map to world matrix is not invertible");
            }
        }

        protected void SetUpdated()
        {
            lastUpdateIndex++;
        }

        /// <summary>
        /// Returns the rectangle ([xMin,yMin],[xMax,xMax]) containing non-default cell values
        /// </summary>
        /// <param name="xMax"></param>
        /// <param name="yMax"></param>
        /// <param name="xMin"></param>
        /// <param name="yMin"></param>
        /// <returns></returns>
        public bool GetMapExtends(out int xMax, out int yMax, out int xMin, out int yMin)
        {
            int lowerStart = -1;
            int upperStart = 10000;

            int xMaxTemp = lowerStart;
            int yMaxTemp = lowerStart;
            int xMinTemp = upperStart;
            int yMinTemp = upperStart;

            for (int x = 0; x < Dimensions.X; ++x)
            {
                for (int y = 0; y < Dimensions.Y; ++y)
                {
                    if (mapArray[y * Dimensions.X + x].Value != 0.0f)
                    {
                        if (x > xMaxTemp)
                        {
                            xMaxTemp = x;
                        }

                        if (x < xMinTemp)
                        {
                            xMinTemp = x;
                        }

                        if (y > yMaxTemp)
                        {
                            yMaxTemp = y;
                        }

                        if (y < yMinTemp)
                        {
                            yMinTemp = y;
                        }
                    }
                }
            }

            if ((xMaxTemp != lowerStart) &&
                (yMaxTemp != lowerStart) &&
                (xMinTemp != upperStart) &&
                (yMinTemp != upperStart)) {

                xMax = xMaxTemp;
                yMax = yMaxTemp;
                xMin = xMinTemp;
                yMin = yMinTemp;

                return true;
            }
            else
            {
                xMax = 0;
                yMax = 0;
                xMin = 0;
                yMin = 0;

                return false;
            }
        }
    }
}
