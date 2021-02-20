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
    [SerializeField] float _NoiseMagnitud = 0.1f;

    private BoidSystemOptions _SystemOptions;

    [Header("Simulation Area Options")]
    [SerializeField] float3 _AreaCenter = float3.zero;
    [SerializeField] float3 _AreaSize = new float3(5, 5, 5);
    [SerializeField] int3 _AreaDivitions = new int3(10,10,10);

    [Header("Render Options")]
    [SerializeField] private UnityEngine.VFX.VisualEffect _PointRederer;
    [SerializeField] private float _ParticleSize = 0.1f;

    [Header("Debug info")]
    [SerializeField] bool _DrawGrid = false;


    private BoidSystem _System;
    private bool _Init;

    private RenderTexture _PointDataTexture;
    private RenderTexture _DirecDataTexture;



    // Start is called before the first frame update
    void Start()
    {
        _Init = false;

        if (_NumberOfBoids % 8 != 0)
            throw new System.Exception("Number of boids MUST be divisible by 8.");
        
        _NumberOfBoids = 1;

        _SystemOptions = new BoidSystemOptions()
        {
            VelocityLimits = _VelocityLimits,
            ObstacleVision = _ObstacleVision,
            ChangeRate = _ChangeRate,
            SeparationRadius = _SeparationRadius,
            CohesionRadius = _CohesionRadius,
            NoiseMagnitude = _NoiseMagnitud
        };

        _System = new BoidSystem(_NumberOfBoids, new SimulationArea(_AreaDivitions, _AreaCenter, _AreaSize), ref _SystemOptions);

        _PointDataTexture = new RenderTexture(_NumberOfBoids, 1, 0, RenderTextureFormat.ARGBHalf, 0);
        _DirecDataTexture = new RenderTexture(_NumberOfBoids, 1, 0, RenderTextureFormat.ARGBHalf, 0);

        _PointDataTexture.enableRandomWrite = true;
        _DirecDataTexture.enableRandomWrite = true;

        _PointDataTexture.Create();
        _DirecDataTexture.Create();

        _PointRederer.SetTexture("PositionMap", _PointDataTexture);
        _PointRederer.SetTexture("DirectionsMap", _DirecDataTexture);

        _PointRederer.SetInt("NumberOfBoids", _NumberOfBoids);
        
        _PointRederer.SetFloat("ParticleSize", _ParticleSize);
        _PointRederer.Reinit();

        _System.InitializeCopyShader(ref _PointDataTexture, ref _DirecDataTexture);

        _Init = true;
    }

    // Update is called once per frame
    void Update()
    {
        UpdateOptionsValues();
        _System.UpdateSystem();
        _System.BakeDataToRenderTexture();

        _PointRederer.SetFloat("ParticleSize", _ParticleSize);

        //Debug.Log($"Test: {_PointRederer.aliveParticleCount}");
    }

    private void UpdateOptionsValues()
    {
        _SystemOptions.VelocityLimits = _VelocityLimits;
        _SystemOptions.ObstacleVision = _ObstacleVision;
        _SystemOptions.ChangeRate = _ChangeRate;
        _SystemOptions.SeparationRadius = _SeparationRadius;
        _SystemOptions.CohesionRadius = _CohesionRadius;
        _SystemOptions.NoiseMagnitude = _NoiseMagnitud;

        _System.UpdateSystemOptions(ref _SystemOptions);
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
