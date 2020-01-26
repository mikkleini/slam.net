using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    public interface IMapLocker
    {
        void LockMap();
        void UnlockMap();
    }
}
