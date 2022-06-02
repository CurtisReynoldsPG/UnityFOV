using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FieldOfView : MonoBehaviour
{

    [SerializeField] LayerMask _targetMask, _obstacleMask;

    [SerializeField] private float _viewRadius;
    [Range(0, 360), SerializeField] private float _viewAngle;
    [SerializeField] private float _meshResolution;
    [SerializeField] private int _edgeResolveIterations;
    [SerializeField] private float _edgeDistanceThreshold;

    [SerializeField] private int _maxTargets = 10;

    private List<Transform> _visibleTargets = new List<Transform>();

    private int _targetsHit = 0;

    [SerializeField] private MeshFilter _meshFilter;
    private Mesh _mesh;

    private Collider[] _targetsInViewRadius;

    public float ViewRadius => _viewRadius;
    public float ViewAngle => _viewAngle;
    public List<Transform> VisibleTargets => _visibleTargets;

    void Awake()
    {
        _targetsInViewRadius = new Collider[_maxTargets];

        _mesh = new Mesh();
        _mesh.name = "View Mesh";
        _meshFilter.mesh = _mesh;
    }

    private void Start()
    {
        StartCoroutine(FindTargetsWithDelay(.1f));
    }

    private void LateUpdate()
    {
        DrawFieldOfView();
    }

    public Vector3 DirFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
        if(!angleIsGlobal)
        {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad),0,Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    void FindVisibleTargets()
    {
        _visibleTargets.Clear();
        Collider[] targetsInViewRadius = Physics.OverlapSphere(transform.position, _viewRadius, _targetMask);

        for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
            Transform target = targetsInViewRadius[i].transform;
            Vector3 dirToTarget = (target.position - transform.position).normalized;
            if (Vector3.Angle(transform.forward, dirToTarget) < _viewAngle / 2)
            {
                float dstToTarget = Vector3.Distance(transform.position, target.position);
                if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, _obstacleMask))
                {
                    _visibleTargets.Add(target);
                }
            }
        }
    }


    void DrawFieldOfView()
    {
        int stepCount = Mathf.RoundToInt(_viewAngle * _meshResolution);
        float stepAngleSize = _viewAngle / stepCount;
        List<Vector3> viewPoints = new List<Vector3>();
        ViewCastInfo oldViewCast = new ViewCastInfo();
        for (int i = 0; i <= stepCount; i++)
        {
            float angle = transform.eulerAngles.y - _viewAngle / 2 + stepAngleSize * i;
            ViewCastInfo newViewCast = ViewCast(angle);

            if (i > 0)
            {
                bool edgeDstThresholdExceeded = Mathf.Abs(oldViewCast.distance - newViewCast.distance) > _edgeDistanceThreshold;
                if (oldViewCast.hit != newViewCast.hit || (oldViewCast.hit && newViewCast.hit && edgeDstThresholdExceeded))
                {
                    EdgeInfo edge = FindEdge(oldViewCast, newViewCast);
                    if (edge.pointA != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointA);
                    }
                    if (edge.pointB != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointB);
                    }
                }
            }

            viewPoints.Add(newViewCast.point);
            oldViewCast = newViewCast;
        }

        int vertexCount = viewPoints.Count + 1;
        Vector3[] vertices = new Vector3[vertexCount];
        int[] triangles = new int[(vertexCount - 2) * 3];

        vertices[0] = Vector3.zero;
        for (int i = 0; i < vertexCount - 1; i++)
        {
            vertices[i + 1] = transform.InverseTransformPoint(viewPoints[i]);

            if (i < vertexCount - 2)
            {
                triangles[i * 3] = 0;
                triangles[i * 3 + 1] = i + 1;
                triangles[i * 3 + 2] = i + 2;
            }
        }

        _mesh.Clear();

        _mesh.vertices = vertices;
        _mesh.triangles = triangles;
        _mesh.RecalculateNormals();
    }

    EdgeInfo FindEdge(ViewCastInfo minViewCast, ViewCastInfo maxViewCast)
    {
        float minAngle = minViewCast.angle;
        float maxAngle = maxViewCast.angle;
        Vector3 minPoint = Vector3.zero;
        Vector3 maxPoint = Vector3.zero;

        for (int i = 0; i < _edgeResolveIterations; i++)
        {
            float angle = (minAngle + maxAngle) / 2;
            ViewCastInfo newViewCast = ViewCast(angle);

            bool edgeDstThresholdExceeded = Mathf.Abs(minViewCast.distance - newViewCast.distance) > _edgeDistanceThreshold;
            if (newViewCast.hit == minViewCast.hit && !edgeDstThresholdExceeded)
            {
                minAngle = angle;
                minPoint = newViewCast.point;
            }
            else
            {
                maxAngle = angle;
                maxPoint = newViewCast.point;
            }
        }

        return new EdgeInfo(minPoint, maxPoint);
    }


    ViewCastInfo ViewCast(float globalAngle)
    {
        Vector3 direction = DirFromAngle(globalAngle, true);
        RaycastHit hit;

        if (Physics.Raycast(transform.position, direction, out hit, _viewRadius, _obstacleMask))
        {
            return new ViewCastInfo(true, hit.point, hit.distance, globalAngle);
        }
        else
        {
            return new ViewCastInfo(false, transform.position + direction * _viewRadius, _viewRadius, globalAngle);
        }
    }

    IEnumerator FindTargetsWithDelay(float delay)
    {
        while (true)
        {
            yield return new WaitForSeconds(delay);
            FindVisibleTargets();
        }
    }
    private void OnDrawGizmos()
    {
        Vector3 viewAngleA = DirFromAngle(-ViewAngle / 2, false);
        Vector3 viewAngleB = DirFromAngle(ViewAngle / 2, false);

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + viewAngleA * ViewRadius);
        Gizmos.DrawLine(transform.position, transform.position + viewAngleB * ViewRadius);

        for (int i = 0; i < _targetsHit; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, _targetsInViewRadius[i].transform.position);
        }

        foreach (Transform target in _visibleTargets)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, target.position);
        }
    }
    public struct ViewCastInfo
    {
        public bool hit;
        public Vector3 point;
        public float distance;
        public float angle;

        public ViewCastInfo(bool hit, Vector3 point, float distance, float angle)
        {
            this.hit = hit;
            this.point = point;
            this.distance = distance;
            this.angle = angle;
        }
    }

    public struct EdgeInfo
    {
        public Vector3 pointA;
        public Vector3 pointB;

        public EdgeInfo(Vector3 pointA, Vector3 pointB)
        {
            this.pointA = pointA;
            this.pointB = pointB;
        }
    }
}

