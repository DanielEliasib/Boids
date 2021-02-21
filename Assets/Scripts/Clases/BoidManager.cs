using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AL.BoidSystem;
using Unity.Mathematics;

using Unity.Collections;

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
    [SerializeField] private Mesh _BoidMesh;
    [SerializeField] private Material _BoidMaterial;

    [Header("Debug info")]
    [SerializeField] bool _DrawGrid = false;

    static readonly int matricesId = Shader.PropertyToID("_Matrices");
    private BoidSystem _System;
    private bool _Init;

    private NativeArray<Matrix4x4> _BoidMatrices;
    private ComputeBuffer _MatrixBuffer;

    // Start is called before the first frame update
    void Start()
    {
        _Init = false;
        
        //_NumberOfBoids = 1;

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

        _System.GetMatrices(ref _BoidMatrices);
        _MatrixBuffer = new ComputeBuffer(_BoidMatrices.Length, 4 * 4 * sizeof(float));

        _BoidMaterial.SetBuffer(matricesId, _MatrixBuffer);
        _Init = true;
    }

    // Update is called once per frame
    void Update()
    {
        UpdateOptionsValues();
        _System.UpdateSystem();

        Debug.Log($"Matrices: {_BoidMatrices.Length}");
        _MatrixBuffer.SetData(_BoidMatrices);

        Graphics.DrawMeshInstancedProcedural(_BoidMesh, 0, _BoidMaterial, new Bounds(_AreaCenter, _AreaSize), _BoidMatrices.Length);

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
        _System?.Dispose();
        _MatrixBuffer?.Dispose();
    }

    private void OnDrawGizmos()
    {
        if (_Init)
        {
            _System.DrawSystem();
            _System.DrawSimulationArea(_DrawGrid);
        }else if (_DrawGrid)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(_AreaCenter, _AreaSize);

        }
    }
}
