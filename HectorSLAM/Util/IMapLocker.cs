using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Util
{
    public interface IMapLocker
    {
        void Lock();
        void Unlock();
    }
}
