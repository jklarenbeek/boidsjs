
export class Boids {

}

export default function createBoids(maxSize = 254, boidCount = 10) {

  maxSize = maxSize | 0;
  boidCount = Math.round(Math.min(maxSize, boidCount))|0;
  const boidsX = new Int32Array(maxSize);
  const boidsY = new Int32Array(maxSize);
  // const boidsZ = new Int8Array(maxSize);
  const boidsVX = new Int32Array(maxSize);
  const boidsVY = new Int32Array(maxSize);

  const default_vec2 = new Int32Array(2);
  function separate(isrc, idst, v = default_vec2) {
    v[0] = (boidsX[isrc] - boidsX[idst])|0;
    v[1] = (boidsY[isrc] - boidsY[idst])|0;

    // Get hypotenuse of triangle
    const d = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
    if ((d < 40) && (d != 0)) { 
      v[0] = v[0]/4;
      v[1] = v[1]/4;
      return true;
    } else {
      return false;
    }
  }
  function separationRule(isrc = 0, v = default_vec2) {
    v[0] = 0; v[1] = 0;
    // not too close please!
    if (boidCount > 0) { // likely
      const w = new Int32Array(2);
      for (let iboid = 0; iboid < boidCount; ++iboid) {
        if (iboid !== isrc) {
          if (separate(isrc, iboid, w)) {
            v[0] = v[0] + w[0];
            v[1] = v[1] + w[1];
          }
        }
      }
    }
    return v;
  }
  function cohesionRule(isrc = 0, v = default_vec2) {
    v[0] = 0; v[1] = 0;
    // match velocity and direction
    if (boidCount > 0) { // likely
      for (let iboid = 0; iboid < boidCount; ++iboid) {
        if (iboid !== isrc) {
          v[0] = v[0] + boidsX[iboid];
          v[1] = v[1] + boidsY[iboid];
        }
      }
      v[0] = v[0] / (boidCount - 1);
      v[1] = v[1] / (boidCount - 1);
      v[0] = v[0] - boidsX[isrc];
      v[1] = v[1] - boidsY[isrc];
      v[0] = v[0] / 100;
      v[1] = v[1] / 100;
    }
    return v;
  }
  function alignmentRule(isrc = 0, v = default_vec2) {
    v[0] = 0; v[1] = 0;
    // match direction and speed
    if (boidCount > 0) { // likely
      for (let iboid = 0; iboid < boidCount; ++iboid) {
        if (iboid !== isrc) {
          v[0] = v[0] + boidsVX[iboid];
          v[1] = v[1] + boidsVY[iboid];
        }
      }
      v[0] = v[0] / (boidCount - 1);
      v[1] = v[1] / (boidCount - 1);
      v[0] = v[0] - boidsVX[isrc];
      v[1] = v[1] - boidsVY[isrc];
      v[0] = v[0] / 8;
      v[1] = v[1] / 8;
    }
    return v;
  }
  function limitVelocity(isrc = 0, v = default_vec2, speedLimit=0.0) {
    v[0] = boidsVX[isrc]|0;
    v[1] = boidsVY[isrc]|0;
    if (speedLimit > 0.0) {
      const speed = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
      if (speed > speedLimit) {
        v[0] = (v[0] / speed) * speedLimit;
        v[1] = (v[1] / speed) * speedLimit;
        return true;
      }
    }
    return false;
  }
  function clampPosition(isrc = 0, vp = default_vec2, vs = default_vec2) {
    vp[0] = boidsX[isrc];
    vp[1] = boidsY[isrc];
    vs[0] = boidsVX[isrc];
    vs[1] = boidsVY[isrc];
  }
  class BoidsImpl extends Boids {
    set count(value) { boidCount = value; }
    get count() { return boidCount; }
    processAll() {
      const v1 = new Int32Array(2);
      const v2 = new Int32Array(2);
      const v3 = new Int32Array(2);
      for (let isrc = 0; isrc < boidCount; ++isrc) {
        cohesionRule(isrc, v1); // go towards center of flock
        alignmentRule(isrc, v2); // match velocity and direction
        separationRule(isrc, v3); // dont hit each other
        boidsVX[isrc] = boidsVX[isrc] + v1[0] + v2[0] + v3[0];
        boidsVY[isrc] = boidsVY[isrc] + v1[1] + v2[1] + v3[1];
        limitVelocity(isrc, v1, 0.0); // dont over heat
        boidsVX[isrc] = v1[0];
        boidsVY[isrc] = v1[1];
        clampPosition(isrc, v1, v2); // cage it
        boidsX[isrc] = v1[0]; // reset position 
        boidsY[isrc] = v1[1]; 
        boidsVX[isrc] = v2[0]; // reset velocity
        boidsVY[isrc] = v2[1];
        boidsX[isrc] = boidsX[isrc] + boidsVX[isrc]; // update position
        boidsY[isrc] = boidsY[isrc] + boidsVY[isrc];
      }
    }
  }

  return new BoidsImpl()

}

/**

struct vec3<T> where T: int8, int16, int32, int64, float32, float64 {
  T x = default(T);
  T y = default(T);
  T z = default(T);
};

operator add(const:vec3 v1, const:vec3 v2)
  returns out:vec3 as v {
  v.x = v1.x + v2.x;
  v.y = v1.y + v2.y;
  v.z = v1.z + v2.z;
}

*/