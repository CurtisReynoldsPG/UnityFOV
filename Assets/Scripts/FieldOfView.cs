using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FieldOfView : MonoBehaviour
{
    [Header("View Settings")]
    [SerializeField] LayerMask _targetMask;
    [SerializeField] LayerMask _obstacleMask;
    [SerializeField] private float _viewRadius;
    [Range(0, 360), SerializeField] private float _viewAngle;

    [Header("Targeting Settings")]
    [SerializeField] private int _maxTargets = 10;
    [SerializeField] private float _targetCheckDelay = .1f; //Setting this to 0.02 will cause it run at the same rate as fixedUpdate

    [Header("Mesh Settings")]
    [SerializeField] private float _meshResolution;
    [SerializeField] private int _edgeResolveIterations;
    [SerializeField] private float _edgeDistanceThreshold;
    [SerializeField] private MeshFilter _meshFilter;


    //Private variables
    private List<Transform> _visibleTargets = new List<Transform>();

    private Vector3 _previousPosition;
    private Quaternion _previousRotation;
    private Mesh _mesh;
    private Collider[] _targetsInViewRadius;
    public List<Transform> VisibleTargets => _visibleTargets;
    private int _targetsHit = 0;

    //Public getters
    public float ViewRadius => _viewRadius;
    public float ViewAngle => _viewAngle;

    //Various initalizations
    void Awake()
    {
        _targetsInViewRadius = new Collider[_maxTargets];

        _mesh = new Mesh();
        _mesh.name = "View Mesh";
        _meshFilter.mesh = _mesh;
    }

    //Simply used to start the main update loop
    private void Start()
    {
        StartCoroutine(FindTargetsWithDelay(_targetCheckDelay));
    }

    //Draws updated mesh
    //Done in late update so its always done after FindTargetsWithDelay when running on the same frame
    private void LateUpdate()
    {
        DrawFieldOfView();

        //Update position and rotation
        _previousPosition = transform.position;
        _previousRotation = transform.rotation;
    }

    //Returns vector direction from angle
    public Vector3 DirFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
        //converts local angle to a global one
        if(!angleIsGlobal)
        {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad),0,Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    //Checks area around object or any visible targets
    void FindVisibleTargets()
    {
        _visibleTargets.Clear();

        //Uses OverlapSphereNonAlloc because it doesn't create gargabe like the standard OverlapSphere does
        _targetsHit = Physics.OverlapSphereNonAlloc(transform.position, _viewRadius, _targetsInViewRadius, _targetMask);


        //Iterate through all hit targets
        for (int i = 0; i < _targetsHit; i++)
        {
            Transform target = _targetsInViewRadius[i].transform;
            Vector3 dirToTarget = (target.position - transform.position).normalized;
            //check if target is in view angle range
            if (Vector3.Angle(transform.forward, dirToTarget) < _viewAngle / 2)
            {
                float dstToTarget = Vector3.Distance(transform.position, target.position);
                //Check if targe is obscured
                if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, _obstacleMask))
                {
                    _visibleTargets.Add(target);
                }
            }
        }
    }

    //Draws mesh to visualize the view cone of the object
    //This isn't actually required for detection and is a completely seperate system, meaning you can alter this / remove it and you'll still have the working detection
    void DrawFieldOfView()
    {
        //check if the transform moved or rotated, if not don't update the mesh
        if (!CheckIfMoved()) return;

        int stepCount = Mathf.RoundToInt(_viewAngle * _meshResolution);
        float stepAngleSize = _viewAngle / stepCount;
        List<Vector3> viewPoints = new List<Vector3>();
        ViewCastInfo oldViewCast = new ViewCastInfo();

        //Casts a ray for each step point determining colision point
        for (int i = 0; i <= stepCount; i++)
        {
            float angle = transform.eulerAngles.y - _viewAngle / 2 + stepAngleSize * i;
            ViewCastInfo newViewCast = ViewCast(angle);

            if (i > 0)
            {
                //Edge detection check to ensure the edge of the mesh is always close to the obstacle to prevent jittering in most cases
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

            //add points to viewpoints
            viewPoints.Add(newViewCast.point);
            oldViewCast = newViewCast;
        }


        //Draws the mesh with the data collected above

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


    //Edge detection steps through min and max angle to determin where the edge of the object between them is
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

    //Raycast function tkaes a globalangle and return information about raycast shot in that direction
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

    //Simple delay feel free to change to fit your needs
    IEnumerator FindTargetsWithDelay(float delay)
    {
        while (true)
        {
            yield return new WaitForSeconds(delay);
            FindVisibleTargets();
        }
    }

    //Checks if the object has moved or rotated since the last call
    private bool CheckIfMoved()
    {
        if (_previousPosition != transform.position || _previousRotation != transform.rotation)
            return true;
        return false;
    }

    //Just some gizmos to allow easier visualization in the game view.
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

    //Struct containing raycast hit data
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

    //Struct containing edge collision data
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

