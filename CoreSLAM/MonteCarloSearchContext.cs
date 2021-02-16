using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using BaseSLAM;

namespace CoreSLAM
{
    /// <summary>
    /// Monte-carlo search context
    /// </summary>
    internal struct MonteCarloSearchContext
    {
        public Thread SearchThread;
        public SignalConcurrentQueue<MonteCarloSearchInput> InputQueue;
        public SignalConcurrentQueue<MonteCarloSearchResult> OutputQueue;
    }
}
