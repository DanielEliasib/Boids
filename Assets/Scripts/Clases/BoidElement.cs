using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

public class BoidSystem
{
    NativeArray<float3> _Positions;
    NativeArray<float3> _Directions;
    NativeArray<float3> _Velocities;

    private int nDivX, nDivY, nDivZ;
    private RectTransform _SimArea;

    private float _SearchRad;

    public BoidSystem(int nOfBoids)
    {
        Initialize(8*1600);
    }

    private void Initialize(int n)
    {
        _Positions = new NativeArray<float3>(n, Allocator.TempJob);
        _Directions = new NativeArray<float3>(n, Allocator.TempJob);
        _Velocities = new NativeArray<float3>(n, Allocator.TempJob);

        InitBoidsJOB _InitJob = new InitBoidsJOB()
        {
            _Pos = _Positions,
            _Dir = _Directions,
            _Vel = _Velocities,
            _Rad = 10
        };

        _InitJob.Schedule(n, 8).Complete();
    }

    private void UpdateSystem()
    {
        
    }
}

public struct SimpleUpdateJOB : IJobParallelFor
{
    public NativeArray<float3> _Pos;
    public NativeArray<float3> _Dir;
    public NativeArray<float3> _Vel;

    public float _SearchRad;

    public void Execute(int index)
    { 
        for(int i = 0; i < _Pos.Length; i++)
        {
            if(i != index)
            {

            }
        }
    }
}

public struct InitBoidsJOB : IJobParallelFor
{
    public NativeArray<float3> _Pos;
    public NativeArray<float3> _Dir;
    public NativeArray<float3> _Vel;

    public float _Rad;

    public void Execute(int index)
    {
        _Pos[index] = UnityEngine.Random.insideUnitSphere * _Rad;
        _Dir[index] = UnityEngine.Random.insideUnitSphere;
        _Vel[index] = UnityEngine.Random.insideUnitSphere;
    }
}