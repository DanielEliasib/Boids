using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

using Unity.Burst;

namespace AL.BoidSystem.Jobs
{
    [BurstCompile]
    public struct BoidSeparationJOB : IJobParallelFor
    {
        //! This will be shared through several JOB. Is it thread safe?
        public NativeArray<float3> _CorrectionForce;

        [ReadOnly] public NativeArray<float3> _OldPosition;
        [ReadOnly] public NativeArray<float3> _OldVelocity;
        [ReadOnly] public NativeArray<int> _LocalCounter;

        //! Simulation Area Data
        [ReadOnly] public NativeParallelMultiHashMap<int, int> _GridToBoidsMap;
        [ReadOnly] public NativeArray<int> _BoidToGridMap;

        [ReadOnly] public BoidSystemOptions _SystemOptions;

        [ReadOnly] public Random _Rand;

        public void Execute(int boidID)
        {
            int gridKey = _BoidToGridMap[boidID];

            if (_LocalCounter[gridKey] > 1)
            {
                float3 newForce = float3.zero;

                int numberOfBoidsToChoose = _Rand.NextInt(1, _LocalCounter[gridKey]);
                var iterator = _GridToBoidsMap.GetValuesForKey(gridKey);
                int counter = 0;

                while (iterator.MoveNext())
                {
                    int otherBoidID = iterator.Current;

                    if (otherBoidID != boidID && counter < numberOfBoidsToChoose)
                    {
                        float3 sep = _OldPosition[otherBoidID] - _OldPosition[boidID];
                        float msqr = sep.x * sep.x + sep.y * sep.y + sep.z * sep.z;
                        if (msqr <= _SystemOptions.SeparationRadiusSqr)
                            newForce += -math.normalize(sep) * _SystemOptions.SeparationRadius - _OldVelocity[boidID];
                        counter++;
                    }

                }

                _CorrectionForce[boidID] += counter > 0 ? newForce : float3.zero;
            }
        }
    }
}