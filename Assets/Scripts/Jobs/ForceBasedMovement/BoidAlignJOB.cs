using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

using Unity.Burst;

namespace AL.BoidSystem.Jobs
{
    [BurstCompile]
    public struct BoidAlignJOB : IJobParallelFor
    {
        //! This will be shared through several JOB. Is it thread safe?
        public NativeArray<float3> _CorrectionForce;

        [ReadOnly] public NativeArray<float3> _LocalPosition;
        [ReadOnly] public NativeArray<float3> _LocalVelocity;
        [ReadOnly] public NativeArray<int> _LocalCounter;

        [ReadOnly] public NativeArray<float3> _OldPosition;
        [ReadOnly] public NativeArray<float3> _OldVelocity;

        //! Simulation Area Data
        [ReadOnly] public NativeArray<int> _BoidToGridMap;

        //: Index goes trought every boid
        public void Execute(int boidID)
        {
            int gridKey = _BoidToGridMap[boidID];

            if (_LocalCounter[gridKey] > 1)
            {
                float3 localVelocity = _LocalVelocity[gridKey] * _LocalCounter[gridKey] - _OldVelocity[boidID];
                localVelocity /= (_LocalCounter[gridKey] - 1);

                _CorrectionForce[boidID] += localVelocity - _OldVelocity[boidID];
            }

        }
    }
}
