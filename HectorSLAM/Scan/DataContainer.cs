using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Scan
{
    public class DataContainer : List<Vector2>
    {
        /// <summary>
        /// TODO What is it ?
        /// </summary>
        public Vector2 Origo { get; private set; }

        /// <summary>
        /// Set from other list
        /// </summary>
        /// <param name="other"></param>
        /// <param name="factor"></param>
        public void SetFrom(DataContainer other, float factor)
        {
            Origo = other.Origo * factor;

            Clear();

            foreach (Vector2 v in other)
            {
                Add(v * factor);
            }
        }
    }
}
