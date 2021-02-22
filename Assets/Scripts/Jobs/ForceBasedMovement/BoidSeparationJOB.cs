using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

namespace AL.BoidSystem.Jobs
{
    public struct BoidSeparationJOB : IJobParallelFor
    {
        //! This will be shared through several JOB. Is it thread safe?
        public NativeArray<float3> _CorrectionForce;

        [ReadOnly] public NativeArray<float3> _OldPosition;
        [ReadOnly] public NativeArray<float3> _OldVelocity;

        //! Simulation Area Data
        [ReadOnly] public NativeMultiHashMap<int, int> _GridToBoidsMap;
        [ReadOnly] public NativeArray<int> _BoidToGridMap;

        [ReadOnly] public BoidSystemOptions _SystemOptions;

        public void Execute(int boidID)
        {
            int gridKey = _BoidToGridMap[boidID];
            var iterator = _GridToBoidsMap.GetValuesForKey(gridKey);

            int counter = 0;

            float3 newForce = float3.zero;

            while (iterator.MoveNext())
            {
                int otherBoidID = iterator.Current;

                if(otherBoidID != boidID)
                {
                    float3 sep = _OldPosition[otherBoidID] - _OldPosition[boidID];
                    float msqr = sep.x * sep.x + sep.y * sep.y + sep.z * sep.z;
                    if (msqr <= _SystemOptions.SeparationRadius * _SystemOptions.SeparationRadius)
                        newForce += -math.normalize(sep) * _SystemOptions.SeparationRadius - _OldVelocity[boidID];
                    counter++;
                }

            }

            _CorrectionForce[boidID] += counter > 0 ? newForce : float3.zero;
        }
    }
}