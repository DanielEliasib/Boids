using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

public struct GenerateRayDirectionsJOB : IJobParallelFor
{
    public NativeArray<float3> _RayDirections;
    [ReadOnly] public float _TotalPoints;
    [ReadOnly] public float golden2;

    public void Execute(int index)
    {
        float phi = math.acos(1.0f - 2 * index / _TotalPoints);
        float theta = math.PI * golden2 * index;

        _RayDirections[index] = new float3(
            math.cos(theta) * math.sin(phi),
            math.sin(theta) * math.sin(phi),
            math.cos(phi)
            );
    }
}
