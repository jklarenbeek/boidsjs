
export class Boids {

}

const CONST_DEFAULT_BOID_RADIUS = 20;
const CONST_DEFAULT_SPEED_LIMIT = 10;

export default function createBoids(viewport = {}, boidCount = 10, maxSize = 254) {

  const structSize = 7;
  const boids = new Int32Array(maxSize * structSize);

  // init boids randomly
  for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
    boids[isrc] = Math.random() * viewport.width;
    boids[isrc + 1] = Math.random() * viewport.height;
    boids[isrc + 2] = Math.sin(Math.random() * Math.PI*2) * CONST_DEFAULT_BOID_RADIUS / 2;
    boids[isrc + 3] = Math.sin(Math.random() * Math.PI * 2) * CONST_DEFAULT_BOID_RADIUS / 2;
    boids[isrc + 4] = CONST_DEFAULT_BOID_RADIUS; // Math.max(3, Math.abs(Math.sin(Math.random() * Math.PI * 2)) * CONST_DEFAULT_SPEED_LIMIT);
    boids[isrc + 5] = 0;
    boids[isrc + 6] = 0;
  }

  class BoidsImpl {
    paint(ctx, size, properties, args) {
      ctx.clearRect(0, 0, size.width, size.height);
      ctx.beginPath();
      for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        let srcx = boids[isrc]|0;
        let srcy = boids[isrc + 1]|0;
        let srcvx = boids[isrc + 2]|0;
        let srcvy = boids[isrc + 3]|0;
        let srcrad = boids[isrc + 4]|0;
        boids[isrc + 5] = srcx;
        boids[isrc + 6] = srcy;
        // let srcsqn = srcvx * srcvx + srcvy * srcvy;
        // let srcmagnitude = Math.pow(srcsqn, .5);
        // let srcangle = Math.atan2(srcvy, srcvy);
        let rule1vx = srcvx;
        let rule1vy = srcvy;

        for (let idst = 0; idst < boidCount * structSize; idst += structSize) {
          if (idst !== isrc) {
            let dstx = boids[idst] | 0;
            let dsty = boids[idst + 1] | 0;
            let dstvx = boids[idst + 2] | 0;
            let dstvy = boids[idst + 3] | 0;
            let dstrad = boids[idst + 4] | 0;

            // separate
            const ldmin = srcrad + dstrad;
            const ldx = dstx - srcx;
            const ldy = dsty - srcy;
            const lsqn = ldx * ldx + ldy * ldy;
            const len = Math.sqrt(lsqn);
            let ldist = Math.abs(len) - ldmin;
            if (len < ldmin) {
              const angle = Math.atan2(ldy, ldx);
              const tx = (Math.cos(angle) * ldmin);
              const ty = (Math.sin(angle) * ldmin);
              const ax = tx ;
              const ay = ty ;
              rule1vx = tx;
              rule1vy = ty;
            }
          }
        }
        
        // limit speed of boid
        srcvx = Math.clampInt(srcvx, -(CONST_DEFAULT_SPEED_LIMIT), CONST_DEFAULT_SPEED_LIMIT);
        srcvy = Math.clampInt(srcvy, -(CONST_DEFAULT_SPEED_LIMIT), CONST_DEFAULT_SPEED_LIMIT);

        // cage boid to outer rectangle
        if (srcvx < 0 && (srcx + srcvx) < srcrad) {
          srcvx = Math.abs(srcvx);
        }
        else if (srcvx > 0 && (viewport.width - (srcx + srcvx)) < srcrad) {
          srcvx = -(srcvx);
        }
        if (srcvy < 0 && (srcy + srcvy) < srcrad) {
          srcvy = Math.abs(srcvy);
        }
        else if (srcvy > 0 && (viewport.height - (srcy + srcvy)) < srcrad) {
          srcvy = -(srcvy);
        }

        srcx += srcvx;
        srcy += srcvy;

        boids[isrc] = srcx;
        boids[isrc + 1] = srcy;
        boids[isrc + 2] = srcvx;
        boids[isrc + 3] = srcvy;
        boids[isrc + 4] = srcrad;

        ctx.moveTo(srcx, srcy);
        ctx.arc(srcx, srcy, srcrad, 0, 2 * Math.PI, false);
        ctx.moveTo(srcx, srcy);
        //ctx.lineTo(rule1vx, rule1vy);
        // ctx.moveTo(srcx, srcy);
        ctx.lineTo((rule1vx), (srcy + rule1vy)*1.5);
      }
      ctx.fillStyle = 'orange';
      ctx.fill();
      ctx.lineWidth = 1;
      ctx.strokeStyle = '#003300';
      ctx.stroke();
    }
  }

  return new BoidsImpl();
}



/**


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

  for (let isrc = 0; isrc < boidCount; ++isrc) {
    boidsX[isrc] = Math.round(Math.random() * 500);
    boidsY[isrc] = Math.round(Math.random() * 500);
    boidsVX[isrc] = Math.round(Math.random() * 5);
    boidsVY[isrc] = Math.round(Math.random() * 5);
  }

  const default_vec2 = new Int32Array(2);
  function separate(isrc, idst, v = default_vec2) {
    v[0] = (boidsX[isrc] - boidsX[idst])|0;
    v[1] = (boidsY[isrc] - boidsY[idst])|0;

    // Get hypotenuse of triangle
    const d = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
    if ((d < boidRadius*2) && (d != 0)) { 
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
  function clampPosition(isrc = 0, radius=0, width=0, height=0, vp = default_vec2, vs = default_vec2) {
    const x = vp[0] = boidsX[isrc];
    const y = vp[1] = boidsY[isrc];
    const vx = vs[0] = Math.min(boidsVX[isrc], radius);
    const vy = vs[1] = Math.min(boidsVY[isrc], radius);
    if (x < radius * 2) {
      vs[0] = ((radius - x) / radius) * vx;
    }
    else if (x > width - radius * 2) {
      vs[0] = ((radius - (width - x)) / radius) * vx;
    }
    if (y < radius * 2) {
      vs[1] = ((radius - y) / radius) * vy;
    }
    else if (y > height - radius * 2) {
      vs[1] = ((radius - (height - y)) / radius) * vy;
    }
    vp[0] += vs[0];
    vp[1] += vs[1];
  }

  const boidRadius = 20;

  class BoidsImpl extends Boids {
    set count(value) { boidCount = value; }
    get count() { return boidCount; }
    processAll(width, height) {
      const v1 = new Int32Array(2);
      const v2 = new Int32Array(2);
      const v3 = new Int32Array(2);
      for (let isrc = 0; isrc < boidCount; ++isrc) {
        cohesionRule(isrc, v1); // go towards center of flock
        alignmentRule(isrc, v2); // match velocity and direction
        separationRule(isrc, v3); // dont hit each other
        boidsVX[isrc] = boidsVX[isrc] + v1[0] + v2[0] + v3[0];
        boidsVY[isrc] = boidsVY[isrc] + v1[1] + v2[1] + v3[1];
        limitVelocity(isrc, v1, 10.0); // dont over heat
        boidsVX[isrc] = v1[0];
        boidsVY[isrc] = v1[1];
        clampPosition(isrc, boidRadius, width, height, v1, v2); // cage it
        boidsX[isrc] = v1[0]; // set position 
        boidsY[isrc] = v1[1]; 
        boidsVX[isrc] = v2[0]; // set velocity
        boidsVY[isrc] = v2[1];
      }
    }
    paint(ctx, size, properties, args) {
      this.processAll(size.width, size.height);
      ctx.beginPath();
      for (let isrc = 0; isrc < boidCount; ++isrc) {
        ctx.moveTo(boidsX[isrc], boidsY[isrc]);
        ctx.arc(boidsX[isrc], boidsY[isrc], boidRadius, 0, 2 * Math.PI, false);
      }
      ctx.fillStyle = 'green';
      ctx.fill();
      ctx.lineWidth = 5;
      ctx.strokeStyle = '#003300';
      ctx.stroke();
    }

  }

  return new BoidsImpl()

}

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