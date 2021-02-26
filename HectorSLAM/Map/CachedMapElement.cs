using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    /// <summary>
    /// Cached map element
    /// </summary>
    public struct CachedMapElement
    {
        /// <summary>
        /// Probability
        /// </summary>
        public float Value;

        /// <summary>
        /// Cache index
        /// </summary>
        public int Index;
    }
}
