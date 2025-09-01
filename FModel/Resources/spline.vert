#version 460 core

// yeeted from minshu https://github.com/FabianFG/CUE4Parse/commit/61cef25b8eef4160651ee41e2b1ceefc5135803f

struct GpuSplineMeshParams {
    int ForwardAxis;
    float SplineBoundaryMin;
    float SplineBoundaryMax;
    bool bSmoothInterpRollScale;

    vec3 MeshOrigin;
    vec3 MeshBoxExtent;

    vec3 StartPos;
    float StartRoll;
    vec3 StartTangent;
    vec2 StartScale;
    vec2 StartOffset;
    vec3 EndPos;
    float EndRoll;
    vec3 EndTangent;
    vec2 EndScale;
    vec2 EndOffset;

    vec3 SplineUpDir;
};

layout(std430, binding = 3) buffer SplineParameters
{
    GpuSplineMeshParams uSplineParameters[];
};

uniform bool uIsSpline;

vec3 getSafeNormal(vec3 vector) {
    float squareSum = dot(vector, vector);

    if (squareSum == 1.0) {
        return vector;
    }

    if (squareSum < 1e-8) {
        return vec3(0.0); // Return a zero vector
    }

    // Calculate the scale factor to normalize the vector
    float scale = inversesqrt(squareSum);
    return vector * scale;
}

float GetAxisValueRef(int forwardAxis, vec3 pos)
{
    switch (forwardAxis)
    {
        case 0: return pos.x;
        case 1: return pos.y;
        case 2: return pos.z;
        default: return 0;
    }
}

void SetAxisValueRef(int forwardAxis, inout vec3 pos, float v)
{
    switch (forwardAxis)
    {
        case 0: pos.x = v; break;
        case 1: pos.y = v; break;
        case 2: pos.z = v; break;
    }
}

vec3 SplineEvalTangent(GpuSplineMeshParams params, float a)
{
    vec3 c = (6 * params.StartPos) + (3 * params.StartTangent) + (3 * params.EndTangent) - (6 * params.EndPos);
    vec3 d = (-6 * params.StartPos) - (4 * params.StartTangent) - (2 * params.EndTangent) + (6 * params.EndPos);
    vec3 e = params.StartTangent;

    float a2 = a * a;

    return (c * a2) + (d * a) + e;
}

vec3 SplineEvalDir(GpuSplineMeshParams params, float a)
{
    return getSafeNormal(SplineEvalTangent(params, a));
}

vec3 SplineEvalPos(GpuSplineMeshParams params, float a)
{
    float a2 = a * a;
    float a3 = a2 * a;

    return (((2 * a3) - (3 * a2) + 1) * params.StartPos) + ((a3 - (2 * a2) + a) * params.StartTangent) + ((a3 - a2) * params.EndTangent) + (((-2 * a3) + (3 * a2)) * params.EndPos);
}

vec3 ComputeRatioAlongSpline(GpuSplineMeshParams params, float distanceAlong)
{
    float alpha = 0.0;
    float minT = 0.0;
    float maxT = 1.0;

    const float SmallNumber = 1e-8;
    bool bHasCustomBoundary = abs(params.SplineBoundaryMin - params.SplineBoundaryMax) > SmallNumber;
    if (bHasCustomBoundary)
    {
        float splineLength = params.SplineBoundaryMax - params.SplineBoundaryMin;
        if (splineLength > 0)
        {
            alpha = (distanceAlong - params.SplineBoundaryMin) / splineLength;
        }

        float boundMin = GetAxisValueRef(params.ForwardAxis, params.MeshOrigin - params.MeshBoxExtent);
        float boundMax = GetAxisValueRef(params.ForwardAxis, params.MeshOrigin + params.MeshBoxExtent);

        float boundMinT = (boundMin - params.SplineBoundaryMin) / (params.SplineBoundaryMax - params.SplineBoundaryMin);
        float boundMaxT = (boundMax - params.SplineBoundaryMin) / (params.SplineBoundaryMax - params.SplineBoundaryMin);

        const float MaxSplineExtrapolation = 4.0;
        minT = max(-MaxSplineExtrapolation, boundMinT);
        maxT = min(boundMaxT, MaxSplineExtrapolation);
    }
    else
    {
        float meshMinZ = GetAxisValueRef(params.ForwardAxis, params.MeshOrigin) - GetAxisValueRef(params.ForwardAxis, params.MeshBoxExtent);
        float meshRangeZ = 2 * GetAxisValueRef(params.ForwardAxis, params.MeshBoxExtent);

        if (meshRangeZ > SmallNumber) {
            alpha = (distanceAlong - meshMinZ) / meshRangeZ;
        }
    }
    return vec3(alpha, minT, maxT);
}

mat4 CalcSliceTransformAtSplineOffset(GpuSplineMeshParams params, vec3 computed)
{
    float alpha = computed.x;
    float minT = computed.y;
    float maxT = computed.z;

    float hermiteAlpha = params.bSmoothInterpRollScale ? smoothstep(0.0, 1.0, alpha) : alpha;

    vec3 splinePos;
    vec3 splineDir;
    if (alpha < minT)
    {
        vec3 startTangent = SplineEvalTangent(params, minT);
        splinePos = SplineEvalPos(params, minT) + (startTangent * (alpha - minT));
        splineDir = getSafeNormal(startTangent);
    }
    else if (alpha > maxT)
    {
        vec3 endTangent = SplineEvalTangent(params, maxT);
        splinePos = SplineEvalPos(params, maxT) + (endTangent * (alpha - maxT));
        splineDir = getSafeNormal(endTangent);
    }
    else
    {
        splinePos = SplineEvalPos(params, alpha);
        splineDir = SplineEvalDir(params, alpha);
    }

    // base
    vec3 baseXVec = getSafeNormal(cross(params.SplineUpDir, splineDir));
    vec3 baseYVec = getSafeNormal(cross(splineDir, baseXVec));

    // Offset the spline by the desired amount
    vec2 sliceOffset = mix(params.StartOffset, params.EndOffset, hermiteAlpha);
    splinePos += sliceOffset.x * baseXVec;
    splinePos += sliceOffset.y * baseYVec;

    // Apply Roll
    float useRoll = mix(params.StartRoll, params.EndRoll, hermiteAlpha);
    float cosAng = cos(useRoll);
    float sinAng = sin(useRoll);
    vec3 xVec = (cosAng * baseXVec) - (sinAng * baseYVec);
    vec3 yVec = (cosAng * baseYVec) + (sinAng * baseXVec);

    // Find Scale
    vec2 useScale = mix(params.StartScale, params.EndScale, hermiteAlpha);

    // Build overall transform
    mat4 sliceTransform = mat4(0);
    vec3 scale;
    switch (params.ForwardAxis) {
        case 0:
        sliceTransform[0] = vec4(splineDir, 0.0);
        sliceTransform[1] = vec4(xVec, 0.0);
        sliceTransform[2] = vec4(yVec, 0.0);
        sliceTransform[3] = vec4(splinePos, 1.0);
        scale = vec3(1.0, useScale.x, useScale.y);
        break;
        case 1:
        sliceTransform[0] = vec4(yVec, 0.0);
        sliceTransform[1] = vec4(splineDir, 0.0);
        sliceTransform[2] = vec4(xVec, 0.0);
        sliceTransform[3] = vec4(splinePos, 1.0);
        scale = vec3(useScale.y, 1.0, useScale.x);
        break;
        case 2:
        sliceTransform[0] = vec4(xVec, 0.0);
        sliceTransform[1] = vec4(yVec, 0.0);
        sliceTransform[2] = vec4(splineDir, 0.0);
        sliceTransform[3] = vec4(splinePos, 1.0);
        scale = vec3(useScale.x, useScale.y, 1.0);
        break;
    }

    mat4 scaleMatrix = mat4(
    vec4(scale.x, 0.0, 0.0, 0.0),
    vec4(0.0, scale.y, 0.0, 0.0),
    vec4(0.0, 0.0, scale.z, 0.0),
    vec4(0.0, 0.0, 0.0, 1.0)
    );

    return sliceTransform * scaleMatrix;
}
