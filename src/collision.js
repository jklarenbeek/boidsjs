
function cross(points, b, c, normal) {
  const bx = b.x;
  const by = b.y;
  const cyby = c.y - by;
  const cxbx = c.x - bx;
  const pa = points.a;
  const pb = points.b;
  const pc = points.c;
    return !(
      (((pa.x - bx) * cyby - (pa.y - by) * cxbx) * normal < 0) ||
      (((pb.x - bx) * cyby - (pb.y - by) * cxbx) * normal < 0) ||
      (((pc.x - bx) * cyby - (pc.y - by) * cxbx) * normal < 0));
}

function trianglesIntersect3(t0, t1) {
  const normal0 = (t0.b.x-t0.a.x)*(t0.c.y-t0.a.y)-
                (t0.b.y-t0.a.y)*(t0.c.x-t0.a.x);
  const normal1 = (t1.b.x-t1.a.x)*(t1.c.y-t1.a.y)-
                (t1.b.y-t1.a.y)*(t1.c.x-t1.a.x);
                
  return !(cross(t1, t0.a, t0.b, normal0) ||
    cross(t1, t0.b, t0.c, normal0) ||
    cross(t1, t0.c, t0.a, normal0) ||
    cross(t0, t1.a, t1.b, normal1) ||
    cross(t0, t1.b, t1.c, normal1) ||
    cross(t0, t1.c, t1.a, normal1));
}
  