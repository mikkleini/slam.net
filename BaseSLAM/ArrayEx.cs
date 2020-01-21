using System;
using System.Collections.Generic;
using System.Text;

namespace BaseSLAM
{
    /// <summary>
    /// Array extensions
    /// </summary>
    public static class ArrayEx
    {
        /// <summary>
        /// Fill 2-dimensional array
        /// </summary>
        /// <typeparam name="T">Element type</typeparam>
        /// <param name="array">Array</param>
        /// <param name="value">Fill value</param>
        public static void Fill<T>(T[,] array, T value)
        {
            for (int i = 0; i < array.GetLength(0); i++)
            {
                for (int j = 0; j < array.GetLength(1); j++)
                {
                    array[i, j] = value;
                }
            }
        }
    }
}
