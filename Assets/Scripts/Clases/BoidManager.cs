using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AL.BoidSystem;
using Unity.Mathematics;

public class BoidManager : MonoBehaviour
{
    [Header("Boids options"), Tooltip("For better results use number divisible by 8.")]
    [SerializeField] int _NumberOfBoids = 8*300;
    [SerializeField] float2 _VelocityLimits = new float2(0.1f, 5.0f);
    [SerializeField] float _ObstacleVision = 1.0f;
    [SerializeField] float _ChangeRate = 0.1f;
    [SerializeField] float _SeparationRadius = 0.1f;
    [SerializeField] float _CohesionRadius = 1.0f;

    private BoidSystemOptions _SystemOptions;

    [Header("Simulation Area Options")]
    [SerializeField] float3 _AreaCenter = float3.zero;
    [SerializeField] float3 _AreaSize = new float3(5, 5, 5);
    [SerializeField] int3 _AreaDivitions = new int3(10,10,10);

    [Header("Debug info")]
    [SerializeField] bool _DrawGrid = false;


    private BoidSystem _System;
    private bool _Init;


    // Start is called before the first frame update
    void Start()
    {
        _Init = false;

        _SystemOptions = new BoidSystemOptions()
        {
            VelocityLimits = _VelocityLimits,
            ObstacleVision = _ObstacleVision,
            ChangeRate = _ChangeRate,
            SeparationRadius = _SeparationRadius,
            CohesionRadius = _CohesionRadius
        };

        _System = new BoidSystem(_NumberOfBoids, new SimulationArea(_AreaDivitions, _AreaCenter, _AreaSize), ref _SystemOptions);
        
        _Init = true;
    }

    // Update is called once per frame
    void Update()
    {
        UpdateOptionsValues();
        _System.UpdateSystem();
    }

    private void UpdateOptionsValues()
    {
        _SystemOptions.VelocityLimits = _VelocityLimits;
        _SystemOptions.ObstacleVision = _ObstacleVision;
        _SystemOptions.ChangeRate = _ChangeRate;
        _SystemOptions.SeparationRadius = _SeparationRadius;
        _SystemOptions.CohesionRadius = _CohesionRadius;
    }

    private void OnDestroy()
    {
        _System.Dispose();
    }

    private void OnDrawGizmos()
    {
        if (_Init)
        {
            _System.DrawSystem();
            _System.DrawSimulationArea(_DrawGrid);
        }
    }
}
