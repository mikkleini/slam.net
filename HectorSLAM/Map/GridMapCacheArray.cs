using System;
using System.Collections.Generic;
using System.Drawing;
using System.Runtime.CompilerServices;
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
        private readonly CachedMapElement[] array = null;

        /// <summary>
        /// The cache iteration index value
        /// </summary>
        private int currentIndex = 0;

        /// <summary>
        /// The dimensions of the array
        /// </summary>
        public Point Dimensions { get; init; }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="dimensions">Dimensionss</param>
        public GridMapCacheArray(Point dimensions)
        {
            Dimensions = dimensions;

            int size = Dimensions.X * Dimensions.Y;
            array = new CachedMapElement[size];
            for (int x = 0; x < size; ++x)
            {
                array[x].index = -1;
            }
        }

        /// <summary>
        /// Resets/deletes the cached data
        /// </summary>
        public void ResetCache()
        {
            currentIndex++;
        }

        /// <summary>
        /// Checks whether cached data for coordinates is available. If this is the case, writes data into value.
        /// </summary>
        /// <param name="index">Array index</param>
        /// <param name="value">Item value</param>
        /// <returns>true if data at index exists</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        /// <summary>
        /// Caches float value val for coordinates coords.
        /// </summary>
        /// <param name="index">Array index</param>
        /// <param name="value">Item value</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CacheData(int index, float value)
        {
            array[index].index = currentIndex;
            array[index].value = value;
        }      
    }
}
