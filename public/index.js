const mathi32_MULTIPLIER = 10000;

const mathi32_abs = Math.abs;

const mathi32_PI = (Math.PI * mathi32_MULTIPLIER)|0;
const mathi32_PI41 = ((4 / Math.PI) * mathi32_MULTIPLIER)|0;
const mathi32_PI42 = ((4 / (Math.PI * Math.PI)) * mathi32_MULTIPLIER)|0;

const mathf64_abs = Math.abs;

const mathf64_sqrt = Math.sqrt;
const mathf64_sin = Math.sin;
const mathf64_cos = Math.cos;
const mathf64_atan2 = Math.atan2;
const mathf64_asin = Math.asin;
const mathf64_max = Math.max;

const mathf64_random = Math.random;

const mathf64_PI = +Math.PI;

let random_seed = mathi32_abs(performance.now() ^ (+mathf64_random() * Number.MAX_SAFE_INTEGER));

class vec2i32 {
  constructor(x = 0, y = 0) {
    this.x = x|0;
    this.y = y|0;
  }
}

const def_vec2i32 = Object.freeze(Object.seal(new vec2i32()));

function float64_hypot(dx = 0.0, dy = 0.0) {
  return +mathf64_sqrt(+(+(+dx * +dx) + +(+dy * +dy)));
}

function float64_theta(x = 0.0, y = 0.0) {
  return +mathf64_atan2(+y, +x);
  /*
    // alternative was faster, but not anymore.
    // error < 0.005
    y = +y;
    x = +x;
    if (x == 0.0) {
      if (y > 0.0) return +(Math.PI / 2.0);
      if (y == 0.0) return 0.0;
      return +(-Math.PI / 2.0);
    }

    const z = +(y / x);
    var atan = 0.0;
    if (+Math.abs(z) < 1.0) {
      atan = +(z / (1.0 + 0.28 * z * z));
      if (x < 0.0) {
        if (y < 0.0) return +(atan - Math.PI);
        return +(atan + Math.PI);
      }
    }
    else {
      atan = +(Math.PI / 2.0 - z / (z * z + 0.28));
      if (y < 0.0) return +(atan - Math.PI);
    }
    return +(atan);
  */
}

/* eslint-disable one-var-declaration-per-line */

class vec2f64 {
  constructor(x = 0.0, y = 0.0) {
    this.x = +x;
    this.y = +y;
  }
}

//#region -- object oriented implementation --

//#region class pure primitive vector operators

vec2f64.prototype.neg = function _vec2f64__neg() {
  return new vec2f64(+(-(+this.x)), +(-(+this.y)));
};

vec2f64.prototype.add = function _vec2f64__add(vector = def_vec2f64) {
  return new vec2f64(+(+this.x + +vector.x), +(+this.y + +vector.y));
};
vec2f64.prototype.adds = function _vec2f64__adds(scalar = 0.0) {
  return new vec2f64(+(+this.x + +scalar), +(+this.y + +scalar));
};

vec2f64.prototype.sub = function _vec2f64__sub(vector = def_vec2f64) {
  return new vec2f64(+(+this.x - +vector.x), +(+this.y - +vector.y));
};
vec2f64.prototype.subs = function _vec2f64__subs(scalar = 0.0) {
  return new vec2f64(+(+this.x - +scalar), +(+this.y - +scalar));
};

vec2f64.prototype.mul = function _vec2f64__mul(vector = def_vec2f64) {
  return new vec2f64(+(+this.x * +vector.x), +(+this.y * +vector.y));
};
vec2f64.prototype.muls = function _vec2f64__muls(scalar = 0.0) {
  return new vec2f64(+(+this.x * +scalar), +(+this.y * +scalar));
};

vec2f64.prototype.div = function _vec2f64__div(vector = def_vec2f64) {
  return new vec2f64(+(+this.x / +vector.x), +(+this.y / +vector.y));
};
vec2f64.prototype.divs = function _vec2f64__divs(scalar = 0.0) {
  return new vec2f64(+(+this.x / +scalar), +(+this.y / +scalar));
};

//#endregion

//#region class impure primitive vector operators
vec2f64.prototype.ineg = function _vec2f64__ineg() {
  this.x = +(-(+this.x));
  this.y = +(-(+this.y));
  return this;
};

vec2f64.prototype.iadd = function _vec2f64__iadd(vector = def_vec2f64) {
  this.x += +vector.x;
  this.y += +vector.y;
  return this;
};
vec2f64.prototype.iadds = function _vec2f64__iadds(value = 0.0) {
  this.x += +value;
  this.y += +value;
  return this;
};

vec2f64.prototype.isub = function _vec2f64__isub(vector = def_vec2f64) {
  this.x -= +vector.x;
  this.y -= +vector.y;
  return this;
};
vec2f64.prototype.isubs = function _vec2f64__isubs(value = 0.0) {
  this.x -= +value;
  this.y -= +value;
  return this;
};

vec2f64.prototype.imul = function _vec2f64__imul(vector = def_vec2f64) {
  this.x *= +vector.x;
  this.y *= +vector.y;
  return this;
};
vec2f64.prototype.imuls = function _vec2f64__imuls(value = 0.0) {
  this.x *= +value;
  this.y *= +value;
  return this;
};

vec2f64.prototype.idiv = function _vec2f64__idiv(vector = def_vec2f64) {
  this.x /= +vector.x;
  this.y /= +vector.y;
  return this;
};
vec2f64.prototype.idivs = function _vec2f64__idivs(value = 0.0) {
  this.x /= +value;
  this.y /= +value;
  return this;
};

//#endregion

//#region class vector products
vec2f64.prototype.mag2 = function _vec2f64__mag2() {
  return +(+(+this.x * +this.x) + +(+this.y * +this.y));
};
vec2f64.prototype.mag = function _vec2f64__mag() {
  return +mathf64_sqrt(+this.mag2());
};

vec2f64.prototype.dot = function _vec2f64__dot(vector = def_vec2f64) {
  return +(+(+this.x * +vector.x) + +(+this.y * +vector.y));
};

/**
 * Returns the cross-product of two vectors
 *
 * @param {vec2f64} vector B
 * @returns {double} The cross product of two vectors
 */
vec2f64.prototype.cross = function _vec2f64__cross(vector = def_vec2f64) {
  return +(+(+this.x * +vector.y) - +(+this.y * +vector.x));
};

/**
 * Returns the cross-product of three vectors
 *
 * You can determine which side of a line a point is on
 * by converting the line to hyperplane form (implicitly
 * or explicitly) and then computing the perpendicular
 * (pseudo)distance from the point to the hyperplane.
 *
 * With the crossproduct of two vectors A and B being the vector
 *
 * AxB = (AyBz − AzBy, AzBx − AxBz, AxBy − AyBx)
 * with Az and Bz being zero you are left with the third component of that vector
 *
 *    AxBy - AyBx
 *
 * With A being the vector from point a to b, and B being the vector from point a to c means
 *
 *    Ax = (b[x]-a[x])
 *    Ay = (b[y]-a[y])
 *    Bx = (c[x]-a[x])
 *    By = (c[y]-a[y])
 *
 * giving
 *
 *    AxBy - AyBx = (b[x]-a[x])*(c[y]-a[y])-(b[y]-a[y])*(c[x]-a[x])
 *
 * which is a scalar, the sign of that scalar will tell you wether point c
 * lies to the left or right of vector ab
 *
 * @param {vec2f64} vector B
 * @param {vec2f64} vector C
 * @returns {double} The cross product of three vectors
 *
 */
vec2f64.prototype.cross3 = function _vec2f64__cross3(vector2 = def_vec2f64, vector3 = def_vec2f64) {
  return +(
    +(+(+vector2.x - +this.x) * +(+vector3.y - +this.y))
    - +(+(+vector2.y - +this.y) * +(+vector3.x - +this.x)));
};

/**
 * Returns the angle in radians of its vector
 *
 * Math.atan2(dy, dx) === Math.asin(dy/Math.sqrt(dx*dx + dy*dy))
 *
 * @param {} v Vector
 */
function _vec2f64__theta() {
  return +mathf64_atan2(+this.y, +this.x);
}
vec2f64.prototype.theta = _vec2f64__theta;
vec2f64.prototype.angle = _vec2f64__theta;
vec2f64.prototype.phi = function _vec2__phi() {
  return +mathf64_asin(+this.y / +this.mag());
};

//#endregion

//#region class pure advanced vector functions
vec2f64.prototype.unit = function _vec2f64__unit() {
  return this.divs(+this.mag());
};

vec2f64.prototype.rotn90 = function _vec2f64__rotn90() {
  return new vec2f64(+this.y, +(-(+this.x)));
};
function _vec2f64__rot90() {
  return new vec2f64(+(-(+this.y)), +this.x);
}
vec2f64.prototype.rot90 = _vec2f64__rot90;
vec2f64.prototype.perp = _vec2f64__rot90;

/**
 * Rotates a vector by the specified angle in radians
 *
 * @param {float} r  angle in radians
 * @returns {vec2f64} transformed output vector
 */
vec2f64.prototype.rotate = function _vec2f64__rotate(radians = 0.0) {
  return new vec2f64(
    +(+(+this.x * +mathf64_cos(+radians)) - +(+this.y * +mathf64_sin(+radians))),
    +(+(+this.x * +mathf64_sin(+radians)) + +(+this.y * +mathf64_cos(+radians))),
  );
};
vec2f64.prototype.about = function _vec2f64__about(vector = def_vec2f64, radians = 0.0) {
  return new vec2f64(
    +(+vector.x + +(+(+(+this.x - +vector.x) * +mathf64_cos(+radians))
      - +(+(+this.y - +vector.y) * +mathf64_sin(+radians)))),
    +(+vector.y + +(+(+(+this.x - +vector.x) * +mathf64_sin(+radians))
      + +(+(+this.y - +vector.y) * +mathf64_cos(+radians)))),
  );
};

//#endregion

//#region class impure advanced vector functions
vec2f64.prototype.iunit = function _vec2f64__iunit() {
  return this.idivs(+this.mag());
};

vec2f64.prototype.irotn90 = function _vec2f64__irotn90() {
  this.x = +this.y;
  this.y = +(-(+this.x));
  return this;
};
function _vec2f64__irot90() {
  this.x = +(-(+this.y));
  this.y = +this.x;
  return this;
}
vec2f64.prototype.irot90 = _vec2f64__irot90;
vec2f64.prototype.iperp = _vec2f64__irot90;

vec2f64.prototype.irotate = function _vec2f64__irotate(radians = 0.0) {
  this.x = +(+(+this.x * +mathf64_cos(+radians)) - +(+this.y * +mathf64_sin(+radians)));
  this.y = +(+(+this.x * +mathf64_sin(+radians)) + +(+this.y * +mathf64_cos(+radians)));
  return this;
};
vec2f64.prototype.iabout = function _vec2f64__iabout(vector = def_vec2f64, radians = 0.0) {
  this.x = +(+vector.x + +(+(+(+this.x - +vector.x) * +mathf64_cos(+radians))
    - +(+(+this.y - +vector.y) * +mathf64_sin(+radians))));
  this.y = +(+vector.y + +(+(+(+this.x - +vector.x) * +mathf64_sin(+radians))
    + +(+(+this.y - +vector.y) * +mathf64_cos(+radians))));
  return this;
};

//#endregion

//#endregion

const def_vec2f64 = Object.freeze(Object.seal(vec2f64_new()));
function vec2f64_new(x = 0.0, y = 0.0) { return new vec2f64(+x, +y); }

class vec3f64 {
  constructor(x = 0.0, y = 0.0, z = 0.0) {
    if (x instanceof vec2f64) {
      this.x = +x.x;
      this.y = +x.y;
      this.z = +y;
    }
    else {
      this.x = +x;
      this.y = +y;
      this.z = +z;
    }
  }
}

const def_vec3f64 = Object.freeze(Object.seal(vec3f64_new()));

//#endregion

function vec3f64_new(x = 0.0, y = 0.0, z = 0.0) { return new vec3f64(+x, +y, +z); }

const workletState = Object.freeze(Object.seal({
  init: 0,
  loading: 1,
  preparing: 2,
  running: 3,
  exiting: 4,
  ended: 5,
}));

const CONST_DEFAULT_BOID_RADIUS = 21.5;
const CONST_DEFAULT_SPEED_LIMIT = mathf64_PI / 3;

function initBoidsf(boidsf, count, size, viewport) {

  // init boids randomly
  for (let isrc = 0; isrc < count * size; isrc += size) {
    // x-position
    boidsf[isrc] = +mathf64_random() * viewport.width; // srcx
    // y-position
    boidsf[isrc + 1] = mathf64_random() * viewport.height; // srcy
    // x-velocity
    boidsf[isrc + 2] = +mathf64_sin(mathf64_random() * mathf64_PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
    // y-velocity
    boidsf[isrc + 3] = +mathf64_sin(mathf64_random() * mathf64_PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
    // angle in unsigned radians
    boidsf[isrc + 4] = (mathf64_random() * mathf64_PI * 2) - mathf64_PI;
    // unsigned radiusX or width
    boidsf[isrc + 5] = +mathf64_max(3, mathf64_abs(mathf64_sin((mathf64_random() * mathf64_PI * 2) - mathf64_PI)) * CONST_DEFAULT_BOID_RADIUS);
    boidsf[isrc + 6] = 0;
  }
}

function createBoids(viewport = {}, boidCount = 52, maxSize = 254) {

  const structSize = 7;
  //const boids = new Int32Array(maxSize * structSize);
  const boidsfBuffer1 = new Float64Array(maxSize * structSize); // boids.buffer);
  const boidsfBuffer2 = new Float64Array(maxSize * structSize); // boids.buffer);

  // setup buffer selector
  let isBuffer1 = true;

  initBoidsf(boidsfBuffer1, boidCount, structSize, viewport);

  class BoidsImpl {
    processFrame(options, callback) {

      const vpwidth = options.width|0;
      const vpheight = options.height|0;
      
      // rollup optimization strangeness fix
      const buf1 = isBuffer1;

      // get current buffer
      const cboidsf = buf1 ? boidsfBuffer1 : boidsfBuffer2;
      const nboidsf = buf1 ? boidsfBuffer2 : boidsfBuffer1;

      // clean our canvas and iterate of all boids
      let srcx = 0.0, srcy = 0.0;
      let srcvx = 0.0, srcvy = 0.0;
      let srca = 0.0;
      let srcw = 0.0, srch = 0.0;
      let color = 'blue';

      for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        srcx = +cboidsf[isrc]; // x position
        srcy = +cboidsf[isrc + 1]; // y position
        srcvx = +cboidsf[isrc + 2]; // x velocity
        srcvy = +cboidsf[isrc + 3]; // y velocity
        srca = +cboidsf[isrc + 4]; // angle (derived from vx/vy when mag > 0.0)
        srch = +cboidsf[isrc + 5]; // height/radiusY
        srcw = +(srch / 2.0);  // width/radiusX
        color = 'blue';

        // cage the boid to the outer rectangle
        {
          const maxx = +(+srch * +(mathf64_PI * mathf64_PI));
          const newx = +(+srcx + +srcvx);
          if (srcvx < 0) {
            const rdistx = +(+maxx - +newx);
            if (+rdistx > 0) {
              const distx = +(maxx - rdistx);
              if (+distx < +srch)
                srcvx = +mathf64_abs(srcvx);
              else {
                color = 'red';
                //const t = +(distx / maxx);
                //srcvx = +((1.0 - t) * srcvx);
                //srcvy = +(t * srcvy);
              }
            }
          }
          else if (srcvx > 0) {
            const rdistx = +(newx - (vpwidth - maxx));
            if (+rdistx > 0) {
              const distx = +(maxx - rdistx);
              if (+distx < +srch) {
                srcvx = +(-(srcvx));
              }
              else {
                color = 'red';
              }
            }
          }
          if (srcvy < 0 && (srcy + srcvy) < srch) {
            srcvy = +mathf64_abs(srcvy);
          }
          else if (srcvy > 0 && (vpheight - (srcy + srcvy)) < srch) {
            srcvy = +(-(srcvy));
          }
        }

        //const srcvx = mathf64_cos(newangle) * newmag;
        //const srcvy = mathf64_sin(newangle) * newmag;

        srcx += +srcvx;
        srcy += +srcvy;
        srca = +float64_theta(+srcvx, +srcvy); // +mathf64_atan2(srcvy, srcvx);

        // save boid state
        nboidsf[isrc] = srcx;
        nboidsf[isrc + 1] = srcy;
        nboidsf[isrc + 2] = srcvx;
        nboidsf[isrc + 3] = srcvy;
        nboidsf[isrc + 4] = srca;
        nboidsf[isrc + 5] = srch;

        // draw the boid
        callback(srcx, srcy, srch, srcw, srca, color);
      }

      // we are done processing
      // flip buffers (something funny with rollup...)
      isBuffer1 = buf1 ? false : true;

    }
    paint(ctx, size, properties, args) {

      // rollup optimization strangness fix
      const buf1 = isBuffer1;

      // get current buffer
      const boidsf = buf1 ? boidsfBuffer1 : boidsfBuffer2;
      
      // the view angle of the boid looking forward.
      const viewAngle = 270 * (mathf64_PI / 180);
      const minViewAngle = (-viewAngle) / 2; // -viewingAngle / 2
      const maxViewAngle = (+viewAngle) / 2; // +viewingAngle / 2

      // indexof the other boids
      let idst = 0;
      // indexof and variables for the current boid
      let isrc = 0; let srcx = 0.0, srcy = 0.0, srcvx = 0.0, srcvy = 0.0, srcrad = 0.0;
      // aggregated velocity for the separation rule
      let rule1cnt = 0; let rule1vx = 0.0, rule1vy = 0.0;
      // aggregated velocity of the alignment rule
      let rule2cnt = 0; let rule2vx = 0.0, rule2vy = 0.0;
      // median position of the cohesion rule
      let rule3cnt = 0; let rule3x = 0.0, rule3y = 0.0;
      // direct collision velocity.
      let rule4cnt = 0; let rule4vx = 0.0, rule4vy = 0.0;
      // aggregated velocities of all rules combined.
      let rulescnt = 0; let rulesvx = 0.0; let rulesvy = 0.0;
      
      // clean our canvas and iterate of all boids
      ctx.clearRect(0, 0, size.width, size.height);
      ctx.beginPath();
      for (isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        //#region Process Boids

        // load source boid variables from typed array
        srcx = boidsf[isrc]; // x position
        srcy = boidsf[isrc + 1]; // y position
        srcvx = boidsf[isrc + 2]; // speed x-axis
        srcvy = boidsf[isrc + 3]; // speed y-axis
        srcrad = boidsf[isrc + 5]; // radius (TODO: radius-x and radius-y)
        // get angle of source boid in radians
        const srctheta = +mathf64_atan2(srcvy, srcvx);
        const srcmag = +float64_hypot(srcvx, srcvy);

        // reset separation rule
        rule1cnt = 0; rule1vx = 0.0; rule1vy = 0.0;
        // reset alignment rule
        rule2cnt = 0; rule2vx = 0.0; rule2vy = 0.0;
        // reset cohesion rule
        rule3cnt = 0; rule3x = 0.0; rule3y = 0.0;
        // reset collision detection rule
        rule4cnt = 0; rule4vx = 0.0; rule4vy = 0.0;
        // reset aggregated velocities
        rulesvx = 0.0; rulesvy = 0.0; rulescnt = 0;

        // iterate through all other boids
        for (idst = 0; idst < boidCount * structSize; idst += structSize) {
          if (idst !== isrc) {
            // load the other boid variables
            const dstx = boidsf[idst];
            const dsty = boidsf[idst + 1];
            const dstvx = boidsf[idst + 2];
            const dstvy = boidsf[idst + 3];
            const dstrad = boidsf[idst + 5];

            // calculate basic distance
            const ldmin = srcrad + dstrad;
            const ldx = dstx - srcx;
            const ldy = dsty - srcy;
            const euc2d = +float64_hypot(ldx, ldy);
            const lux = ldx / euc2d;
            const luy = ldy / euc2d;

            // we enter when we are at least within some distance.
            if (euc2d < ldmin * 2) {
              
              // collision detection
              if (euc2d < ldmin) { // TODO: mass and velocity is not correctly transfered.
                //rule4vx += (vx / hyp) / mathf64_PI;
                //rule4vy += (vy / hyp) / mathf64_PI;
                rule4vx += +((lux * -1) * 0.853);
                rule4vy += +((luy * -1) * 0.853);
                rule4cnt++;
                //continue;
              }

              // view angle detection
              const spdy = (size.height - dsty) - (size.height - srcy);
              const spx = ldx * mathf64_cos(srctheta) - spdy * mathf64_sin(srctheta);
              const spy = ldx * mathf64_sin(srctheta) + spdy * mathf64_cos(srctheta);
          
              const spa = mathf64_atan2(-spy, spx);
          
              // within view? apply flocking rules
              if (spa < maxViewAngle && spa > minViewAngle) {
                //const angle = mathf64_atan2(ldy, ldx);
                //const tx = (mathf64_cos(angle) * ldmin * 1.0003);
                //const ty = (mathf64_sin(angle) * ldmin * 1.0003);
                //const sdx = (dstx - (srcx + tx));
                //const sdy = (dsty - (srcy + ty));
                //const ddx = (srcx - (dstx + tx));
                //const ddy = (srcy - (dsty + ty));
                //const vx = ((dstrad - srcrad) * sdx + (dstrad + dstrad) * ddx) / ldmin;
                //const vy = ((dstrad - srcrad) * sdy + (dstrad + dstrad) * ddy) / ldmin;

                // separate
                rule1vx += (srcvx / srcmag + (lux * -1)) / 2;
                rule1vy += (srcvy / srcmag + (luy * -1)) / 2;
                //rule1vx += +(((srcvx) + (+(srcx - dstx) / +euc2d)) / 2.0);
                //rule1vy += +(((srcvy) + (+(srcy - dsty) / +euc2d)) / 2.0);
                rule1cnt++;

              // alignment
              // TODO: add weights to its size.
              const dstmag = float64_hypot(dstvx, dstvy);
              rule2vx += (dstvx / dstmag);
              rule2vy += (dstvy / dstmag);
              rule2cnt++;

              // cohesion
                rule3x += dstx;
                rule3y += dsty;
                rule3cnt++;
              }

            }
        
            // alignment
            //rule2vx += (dstvx);
            //rule2vy += (dstvy);
            //rule2cnt++;

          }
        }
        //#endregion

        //#region aggregate rules
        {
          rulesvx = 0; //srcvx;
          rulesvy = 0; //srcvy;
          rulescnt = 0;
          if (rule1cnt > 0) {
            // separate
            rulesvx += (rule1vx / rule1cnt);
            rulesvy += (rule1vy / rule1cnt);
            rulescnt++;
          }
          if (rule2cnt > 0) {
            // alignment
            rulesvx += (rule2vx / rule2cnt) * 0.13;
            rulesvy += (rule2vy / rule2cnt) * 0.13;
            rulescnt++;
          }
          if (rule3cnt > 0) {
            // cohesion
            const vx = ((rule3x / rule3cnt) - srcx);
            const vy = ((rule3y / rule3cnt) - srcy);
            const nm = float64_hypot(vx, vy);
            rulesvx += (srcvx + (vx / nm)) / mathf64_PI;
            rulesvy += (srcvy + (vy / nm)) / mathf64_PI;
            rulescnt++;
          }
          if (rule4cnt > 0) {
            // collision
            rulesvx = (rule4vx / rule4cnt);
            rulesvy = (rule4vy / rule4cnt);
            rulescnt++;
          }
          if (rulescnt > 0) {
            rulesvx /= rulescnt;
            rulesvy /= rulescnt;
            srcvx += (rulesvx);// * 0.03; // / (mathf64_PI * mathf64_PI));
            srcvy += (rulesvy);// * 0.03; // / (mathf64_PI * mathf64_PI));
          }
        }

        //#endregion

        //#region limit source boid

        // limit speed of boid
        const newmag = +float64_hypot(srcvx, srcvy);
        if (newmag > CONST_DEFAULT_SPEED_LIMIT) {
          srcvx = (srcvx / newmag) * CONST_DEFAULT_SPEED_LIMIT;
          srcvy = (srcvy / newmag) * CONST_DEFAULT_SPEED_LIMIT;
        }

        // cage boid to outer rectangle
        {
          if (srcvx < 0 && (srcx + srcvx) < srcrad) {
            srcvx = +mathf64_abs(srcvx);
          }
          else if (srcvx > 0 && (size.width - (srcx + srcvx)) < srcrad) {
            srcvx = -(srcvx);
          }
          if (srcvy < 0 && (srcy + srcvy) < srcrad) {
            srcvy = +mathf64_abs(srcvy);
          }
          else if (srcvy > 0 && (size.height - (srcy + srcvy)) < srcrad) {
            srcvy = -(srcvy);
          }
        }

        //#endregion

        // update position
        srcx += srcvx;
        srcy += srcvy;

        // save boid state
        boidsf[isrc] = srcx;
        boidsf[isrc + 1] = srcy;
        boidsf[isrc + 2] = srcvx;
        boidsf[isrc + 3] = srcvy;
        boidsf[isrc + 5] = srcrad;

        //#region draw boid
        if (rule4cnt > 0) ctx.fillStyle = 'red';
        else if (rule1cnt > 0) ctx.fillStyle = `green`;
        else if (rule2cnt > 0) ctx.fillStyle = `yellow`;
        else if (rule3cnt > 0) ctx.fillStyle = `purple`;
        else ctx.fillStyle = 'blue';

        ctx.save();

        ctx.translate(srcx, srcy);
        ctx.rotate(mathf64_atan2(srcvy, srcvx));
  
        ctx.beginPath();

        // const boidlength = CONST_DEFAULT_BOID_RADIUS * 3.45;
        // const bdi = boidlength/72
        // ctx.lineTo(-boidlength/4, bdi)
        // ctx.lineTo(-boidlength/3, bdi)
        // ctx.lineTo(-boidlength/2, boidlength/6)
        // ctx.lineTo(-boidlength/2, -boidlength/6)
        // ctx.lineTo(-boidlength/3, -bdi)
        // ctx.lineTo(-boidlength / 4, -bdi)
        
        // ctx.fill();

        //ctx.moveTo(srcx, srcy);
        //ctx.arc(0, 0, srcrad, 0, 2 * mathf64_PI, false);
        ctx.ellipse(0, 0, srcrad, srcrad / 2, 0, 0, mathf64_PI * 2);
        ctx.fill();
        //ctx.moveTo(srcx, srcy);
        //ctx.lineTo(rule1vx, rule1vy);
        //ctx.stroke();
        // ctx.moveTo(srcx, srcy);
        //ctx.lineTo(srcx + rule1vx, srcy + rule1vy);

        // ctx.closePath();

        ctx.restore();
        //#endregion

      }
      // ctx.fillStyle = 'orange';
      // ctx.fill();
      //ctx.lineWidth = 1;
      //ctx.strokeStyle = '#003300';
      //ctx.stroke();
    }
  }

  return new BoidsImpl();
}

function main() {
  const canvas = document.getElementById('boids-canvas');
  const ctx = canvas.getContext('2d');

  const boids = createBoids({ width: canvas.clientWidth, height: canvas.clientHeight });
  requestAnimationFrame(function draw(now) {
    requestAnimationFrame(draw);
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    boids.paint(ctx, { width: canvas.clientWidth, height: canvas.clientHeight });
  });
}

main();
//# sourceMappingURL=index.js.map
