using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

using Unity.Burst;

namespace AL.BoidSystem.Jobs
{
    [BurstCompile]
    public struct GridPreCalculationsJOB : IJobParallelFor
    {
        public NativeArray<float3> _LocalPosition;
        public NativeArray<float3> _LocalVelocity;
        public NativeArray<int> _BoidPerGrid;

        [ReadOnly] public NativeArray<float3> _OldPosition;
        [ReadOnly] public NativeArray<float3> _OldVelocity;

        //! Simulation Area Data
        [ReadOnly] public NativeMultiHashMap<int, int> _GridToBoidsMap;

        public void Execute(int gridID)
        {
            var iterator = _GridToBoidsMap.GetValuesForKey(gridID);

            int counter = 0;

            double3 localVelocity = float3.zero;
            double3 localPosition = float3.zero;

            while (iterator.MoveNext())
            {
                int boidID = iterator.Current;

                localVelocity += _OldVelocity[boidID];
                localPosition += _OldPosition[boidID];
                counter++;
            }

            _LocalPosition[gridID] = (float3)(localPosition/counter);
            _LocalVelocity[gridID] = (float3)(localVelocity/counter);
            _BoidPerGrid[gridID] = counter;
        }
    }
}