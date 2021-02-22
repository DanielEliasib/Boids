using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using UnityEngine;

using Unity.Burst;

namespace AL.BoidSystem.Jobs
{
    [BurstCompile]
    public struct BoidCohesionJOB : IJobParallelFor
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

        public void Execute(int boidID)
        {
            int gridKey = _BoidToGridMap[boidID];

            if(_LocalCounter[gridKey] > 1)
            {
                float3 localPosition = _LocalPosition[gridKey] * _LocalCounter[gridKey] - _OldPosition[boidID];
                localPosition /= (_LocalCounter[gridKey] - 1);

                _CorrectionForce[boidID] += localPosition - _OldPosition[boidID] - _OldVelocity[boidID];
                //_CorrectionForce[boidID] += counter > 0 ? (localPosition / counter - _OldPosition[boidID]) - _OldVelocity[boidID] : float3.zero;
            }
            
        }

    }
}