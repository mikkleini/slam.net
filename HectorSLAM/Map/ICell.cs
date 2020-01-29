using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    public interface ICell
    {
        float Value { get; set; }
        bool IsOccupied { get; }
        bool IsFree { get; }
        void Reset();
    }
}
