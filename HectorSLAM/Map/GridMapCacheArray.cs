using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;

namespace HectorSLAM.Map
{
    /// <summary>
    /// Caches filtered grid map accesses in a two dimensional array of the same size as the map.
    /// </summary>
    public class GridMapCacheArray
    {
        /// <summary>
        /// Array used for caching data.
        /// </summary>
        private CachedMapElement[] array = null;

        /// <summary>
        /// The cache iteration index value
        /// </summary>
        private int currentIndex = 0;

        /// <summary>
        /// The size of the array
        /// </summary>
        public Point Dimensions { get; private set; } = new Point(-1, -1);

        /// <summary>
        /// Constructor
        /// </summary>
        public GridMapCacheArray()
        {
        }

        /// <summary>
        /// Resets/deletes the cached data
        /// </summary>
        public void ResetCache()
        {
            currentIndex++;
        }

        /**
         * Checks wether cached data for coords is available. If this is the case, writes data into val.
         * @param coords The coordinates
         * @param val Reference to a float the data is written to if available
         * @return Indicates if cached data is available
         */
        public bool ContainsCachedData(int index, out float value)
        {
            if (array[index].index == currentIndex)
            {
                value = array[index].value;
                return true;
            }
            else
            {
                value = float.NaN;
                return false;
            }
        }

        /**
         * Caches float value val for coordinates coords.
         * @param coords The coordinates
         * @param val The value to be cached for coordinates.
         */
        public void CacheData(int index, float value)
        {
            array[index].index = currentIndex;
            array[index].value = value;
        }

        /**
         * Sets the map size and resizes the cache array accordingly
         * @param sizeIn The map size.
         */
        public void SetMapSize(Point newDimensions)
        {
            SetArraySize(newDimensions);
        }
    
        /**
        * Creates a cache array of size sizeIn.
        * @param sizeIn The size of the array
        */
        protected void CreateCacheArray(Point newDimensions)
        {
            Dimensions = newDimensions;

            int size = Dimensions.X * Dimensions.Y;

            array = new CachedMapElement[size];

            for (int x = 0; x < size; ++x)
            {
                array[x].index = -1;
            }
        }

        /**
         * Sets a new cache array size
         */
        protected void SetArraySize(Point newDimensions)
        {
            if (Dimensions != newDimensions)
            {
                CreateCacheArray(newDimensions);
            }
        }
    }
}
