using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace HectorSLAM.Util
{
    public interface IHectorDebugInfo
    {
        void SendAndResetData();
        void AddHessianMatrix(Matrix4x4 hessian);
        void AddPoseLikelihood(float lh);
    }
}
