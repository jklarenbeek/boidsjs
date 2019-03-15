
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

const default_vec2 = new Int32Array(2);
function crossme(v2l1, v2l2, v2l3, v2r1, v2r2, normal) {
  const dx = v2r2[0] - v2r1[0];
  const dy = v2r2[1] - v2r1[1];
  return !(
    (((v2l1[0] - v2r1[0]) * dy - (v2l1[1] - v2r1[1]) * dx) * normal < 0) ||
    (((v2l2[0] - v2r1[0]) * dy - (v2l2[1] - v2r1[1]) * dx) * normal < 0) ||
    (((v2l3[0] - v2r1[0]) * dy - (v2l3[1] - v2r1[1]) * dx) * normal < 0));
}
export function trianglesIntersect(v2l1, v2l2, v2l3, v2r1, v2r2, v2r3) {
  const lnorm = (v2l2[0] - v2l1[0]) * (v2l3[1] - v2l1[1])
    - (v2l2[1] - v2l1[1]) * (v2l3[0] - v2l1[0]);
  const rnorm = (v2r2[0] - v2r1[0]) * (v2r3[1] - v2r1[1])
    - (v2r2[1] - v2r1[1]) * (v2r3[0] - v2r1[0]);
  return !(crossme(v2r1, v2r2, v2r3, v2l1, v2l2, lnorm)
    || crossme(v2r1, v2r2, v2r3, v2l2, v2l3, lnorm)
    || crossme(v2r1, v2r2, v2r3, v2l3, v2l1, lnorm)
    || crossme(v2l1, v2l2, v2l3, v2r1, v2r2, rnorm)
    || crossme(v2l1, v2l2, v2l3, v2r2, v2r3, rnorm)
    || crossme(v2l1, v2l2, v2l3, v2r3, v2r1, rnorm));
}

export function triangleIntersect(r1, r2, v1, v2, v3) {
/*
  This function borrowed faithfully from a wonderfl (:3) discussion on
  calculating triangle collision with AABBs on the following blog:
  http://sebleedelisle.com/2009/05/super-fast-trianglerectangle-intersection-test/

  This particular optimization best suits my purposes and was contributed
  to the discussion by someone from http://lab9.fr/
  */

  const l = r1[0];
  const r = r2[0];
  const t = r1[1];
  const b = r2[1];

  const x0 = v1[0];
  const y0 = v1[1];
  const x1 = v2[0];
  const y1 = v2[1];
  const x2 = v3[0];
  const y2 = v3[1];

  const b0 = ((x0 > l) ? 1 : 0) | (((y0 > t) ? 1 : 0) << 1) |
      (((x0 > r) ? 1 : 0) << 2) | (((y0 > b) ? 1 : 0) << 3);
  if (b0 == 3) return true;

  const b1 = ((x1 > l) ? 1 : 0) | (((y1 > t) ? 1 : 0) << 1) |
      (((x1 > r) ? 1 : 0) << 2) | (((y1 > b) ? 1 : 0) << 3);
  if (b1 == 3) return true;

  const b2 = ((x2 > l) ? 1 : 0) | (((y2 > t) ? 1 : 0) << 1) |
      (((x2 > r) ? 1 : 0) << 2) | (((y2 > b) ? 1 : 0) << 3);
  if (b2 == 3) return true;

  let c = 0;
  let m = 0;
  let s = 0;

  const i0 = b0 ^ b1;
  if (i0 != 0) {
      m = ((y1-y0) / (x1-x0))|0;
      c = (y0 -(m * x0))|0;
      if (i0 & 1) { s = m * l + c; if ( s > t && s < b) return true; }
      if (i0 & 2) { s = (t - c) / m; if ( s > l && s < r) return true; }
      if (i0 & 4) { s = m * r + c; if ( s > t && s < b) return true; }
      if (i0 & 8) { s = (b - c) / m; if ( s > l && s < r) return true; }
  }

  const i1 = b1 ^ b2;
  if (i1 != 0) {
      m = (y2-y1) / (x2-x1);
      c = y1 -(m * x1);
      if (i1 & 1) { s = m * l + c; if ( s > t && s < b) return true; }
      if (i1 & 2) { s = (t - c) / m; if ( s > l && s < r) return true; }
      if (i1 & 4) { s = m * r + c; if ( s > t && s < b) return true; }
      if (i1 & 8) { s = (b - c) / m; if ( s > l && s < r) return true; }
  }

  const i2 = b0 ^ b2;
  if (i2 != 0)
  {
      m = (y2-y0) / (x2-x0);
      c = y0 -(m * x0);
      if (i2 & 1) { s = m * l + c; if ( s > t && s < b) return true; }
      if (i2 & 2) { s = (t - c) / m; if ( s > l && s < r) return true; }
      if (i2 & 4) { s = m * r + c; if ( s > t && s < b) return true; }
      if (i2 & 8) { s = (b - c) / m; if ( s > l && s < r) return true; }
  }

  return false;
}
