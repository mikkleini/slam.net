using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    public interface ICell
    {
        int UpdateIndex { get; set; }
        float Value { get; set; }
        bool IsOccupied { get; }
        bool IsFree { get; }
        void Reset();
    }
}
