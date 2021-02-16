using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using BaseSLAM;

namespace CoreSLAM
{
    /// <summary>
    /// Monte-carlo search result
    /// </summary>
    internal struct MonteCarloSearchResult
    {
        public Vector3 pose;
        public int distance;
    }
}
