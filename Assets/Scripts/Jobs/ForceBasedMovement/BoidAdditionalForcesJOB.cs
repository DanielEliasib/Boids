using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

namespace AL.BoidSystem.Jobs
{
    public struct BoidAdditionalForcesJOB : IJobParallelFor
    {
        //! This will be shared through several JOB. Is it thread safe?
        public NativeArray<float3> _CorrectionForce;

        [ReadOnly] public NativeArray<float3> _OldPosition;
        [ReadOnly] public NativeArray<float3> _OldVelocity;
        [ReadOnly] public NativeArray<float3> _InterestPoints;
        
        public void Execute(int boidID)
        {
            float3 correctionForce = float3.zero;

            for(int i = 0; i < _InterestPoints.Length; i++)
            {
                correctionForce += (_InterestPoints[i] - _OldPosition[boidID]) - _OldVelocity[boidID];
            }

            _CorrectionForce[boidID] += correctionForce*0.01f;
        }

    }
}