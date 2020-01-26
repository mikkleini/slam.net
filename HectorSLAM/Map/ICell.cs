using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    public interface ICell
    {
        void Set(float val);
        float GetValue();
        bool IsOccupied();
        bool IsFree();
        void ResetGridCell();
    }
}
