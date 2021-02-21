using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Mathematics;

public class SphereTests : MonoBehaviour
{
    float3[] _SpherePoints;
    [SerializeField, Range(0.0f, 1.0f)] float _Fraction = 0.5f;
    [SerializeField, Range(1, 1000)] int numberOfRays = 100;
    // Start is called before the first frame update
    void Start()
    {
        _SpherePoints = new float3[numberOfRays];
        GenerateRayPoints(ref _SpherePoints);
    }

    // Update is called once per frame
    void Update()
    {
        _SpherePoints = new float3[numberOfRays];
        GenerateRayPoints(ref _SpherePoints);
    }

    private void OnDrawGizmos()
    {
        Color color1 = Color.red;
        Color color2 = Color.blue;

        if (_SpherePoints != null)
        {
            for(int i = 0; i < _SpherePoints.Length*0.5f; i++)
            {
                Gizmos.color = Color.Lerp(color1, color2, (float)i/(_SpherePoints.Length-1.0f));
                Gizmos.DrawWireSphere(_SpherePoints[i], 0.01f);
            }
        }
    }

    public void GenerateRayPoints(ref float3[] points)
    {
        int numberOfRays = points.Length;

        float golden2 = (1.0f + Mathf.Sqrt(5));

        for (int i = 0; i < numberOfRays; i++)
        {
            float index = i + _Fraction;

            float phi = Mathf.Acos(1 - 2 * index / numberOfRays);
            float theta = Mathf.PI*golden2 * index;

            float x = Mathf.Cos(theta) * Mathf.Sin(phi);
            float y = Mathf.Sin(theta) * Mathf.Sin(phi);
            float z = Mathf.Cos(phi);

            points[i] = new float3(x,y,z);
        }
    }
}
