#include "swTriangle.h"

namespace sw {

bool Triangle::intersect(const Ray &r, Intersection &isect) const {
    // 1st: Find the projection of rays origin onto the triangle plane:
    Vec3 e1 = vertices[1] - vertices[0];    //Edge 1
    Vec3 e2 = vertices[2] - vertices[0];    //Edge 2

    Vec3 planeNormal = e1 % e2; // planeNormal by cross product
    float m = -planeNormal * vertices[0]; // Plane equation: n.X + m = 0;
    float t = (planeNormal * r.orig + m) / (-planeNormal * r.dir);  // plane intersection: t = (n.P+m)/(-n.d)
    //Check t
    if ((r.minT <= t) && (r.maxT >= t))
    {
        Vec3 Q = r.orig + t * r.dir; // projection point Q of P onto the triangle plane
        Vec3 coVec = Q - vertices[0];    // vector r coplanar with e1, e2

        //calculate barycentric v, w
        float v = sqrt((e1 % coVec) * (e1 % coVec)) / sqrt(planeNormal * planeNormal);
        float w = sqrt((coVec % e2) * (coVec % e2)) / sqrt(planeNormal * planeNormal);

        if ((v >= 0) && (w >= 0) && ((v+w)<1))
        {
            isect.hitT = t;
            isect.normal = planeNormal;
            isect.normal.normalize();
            isect.frontFacing = (-r.dir *isect.normal) > 0.0f;
            if (!isect.frontFacing) isect.normal = -isect.normal;
            isect.position = Q;
            isect.material = material;
            isect.ray = r;
            return true;
        } else
            return false;

    }
    else
        return false;
    




    
    return false;
}

} // namespace sw
