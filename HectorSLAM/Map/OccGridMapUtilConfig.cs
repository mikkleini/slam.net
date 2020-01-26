using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    public class OccGridMapUtilConfig<ConcreteOccGridMap> : OccGridMapUtil<ConcreteOccGridMap, GridMapCacheArray>
    {
        public OccGridMapUtilConfig(ConcreteOccGridMap gridMap)
            : base(gridMap)
        {
        }
    }
}
