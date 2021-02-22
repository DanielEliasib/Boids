using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

namespace AL.BoidSystem.Jobs
{
    public struct BoidAlignJOB : IJobParallelFor
    {
        //! This will be shared through several JOB. Is it thread safe?
        public NativeArray<float3> _CorrectionForce;

        [ReadOnly] public NativeArray<float3> _OldPosition;
        [ReadOnly] public NativeArray<float3> _OldVelocity;

        //! Simulation Area Data
        [ReadOnly] public NativeMultiHashMap<int, int> _GridToBoidsMap;
        [ReadOnly] public NativeArray<int> _BoidToGridMap;
        [ReadOnly] public SimulationArea _SimArea;

        //: Index goes trought every boid
        public void Execute(int boidID)
        {
            int gridKey = _BoidToGridMap[boidID];
            var iterator = _GridToBoidsMap.GetValuesForKey(gridKey);

            int counter = 0;

            float3 localVelocity = float3.zero;

            while (iterator.MoveNext())
            {
                int otherBoidID = iterator.Current;

                localVelocity += otherBoidID != boidID?_OldVelocity[otherBoidID] : float3.zero;
                counter += otherBoidID != boidID? 1 : 0;
            }
            
            _CorrectionForce[boidID] += counter > 0? (localVelocity/counter - _OldVelocity[boidID]) : float3.zero;
        }
    }
}