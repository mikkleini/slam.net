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
    /// Monte-carlo search input
    /// </summary>
    internal struct MonteCarloSearchInput
    {
        public ScanCloud cloud;
        public Vector3 startPose;
    }
}
