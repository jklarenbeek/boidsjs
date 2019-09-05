import {
  mathf64_PI,
  mathf64_abs,
  mathf64_random,
  mathf64_atan2,
  mathf64_cos,
  mathf64_max,
  mathf64_sin,

  float64_sqrt,
  float64_hypot2,
  float64_hypot,
  float64_dot,
  float64_theta
} from 'futilsjs';

// "https://github.com/lovasoa/expectation-maximization"

const CONST_DEFAULT_BOID_RADIUS = 21.5;
const CONST_DEFAULT_SPEED_LIMIT = mathf64_PI / 3;

const CONST_DEFAULT_BOID_STRUCT_SIZE = 8;

class Boids  {
  static createArray(capacity, arrayType = Array) {
    return new arrayType(CONST_DEFAULT_BOID_STRUCT_SIZE * array.length);
  }

  constructor(array) {
    this._data = array
    this._capacity = array.length / CONST_DEFAULT_BOID_STRUCT_SIZE;
    this._pointer = 0;
  }

  get data() { return this._data; }
  get pointer() { return this._pointer; }
  set pointer(value) { this._pointer = value; }

  capacity() { return this._capacity; }
  current() { return this.pointer / CONST_DEFAULT_BOID_STRUCT_SIZE; }

  reset() { this.pointer = 0; }
  next() { this.pointer += CONST_DEFAULT_BOID_STRUCT_SIZE; }
  isLast() { return this.pointer >= this.data.length; }

  get x() { this.data[this.pointer + 0]; }
  get y() { this.data[this.pointer + 1]; }
  get vx() { this.data[this.pointer + 2]; }
  get vy() { this.data[this.pointer + 3]; }
  get angle() { this.data[this.pointer + 4]; }
  get width() { this.data[this.pointer + 5]; }
  get height() { this.data[this.pointer + 6]; }
  get mass() { this.data[this.pointer + 7]; }

  set x(value) { this.data[this.pointer + 0] = value; }
  set y(value) { this.data[this.pointer + 1] = value; }
  set vx(value) { this.data[this.pointer + 2] = value; }
  set vy(value) { this.data[this.pointer + 3] = value; }
  set angle(value) { this.data[this.pointer + 4] = value; }
  set width(value) { this.data[this.pointer + 5] = value; }
  set height(value) { this.data[this.pointer + 6] = value; }
  set mass(value) { this.data[this.pointer + 7] = value; }

  init(viewport) {
    for (this.reset(); !this.isLast(); this.next()) {
      this.x = mathf64_random() * viewport.width;
      this.y = mathf64_random() * viewport.height; 
      this.vx = mathf64_sin(mathf64_random() * mathf64_PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
      this.vy = mathf64_sin(mathf64_random() * mathf64_PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
      this.angle = (mathf64_random() * mathf64_PI * 2) - mathf64_PI;
      // unsigned radiusX or width
      this.height = mathf64_max(3, mathf64_abs(mathf64_sin((mathf64_random() * mathf64_PI * 2) - mathf64_PI)) * CONST_DEFAULT_BOID_RADIUS);
      this.width = this.height / 2;
      this.mass = this.height * this.width * 0.639;
    }
  }

  process(viewport, callback) {
    const other = new Boids(this.data);
    for (this.reset(); !this.isLast(); this.next()) {
      for (other.reset(); !other.isLast(); other.next()) {
        // compute distance from each other
        const distx = this.x - other.x;
        const disty = this.y - other.y;
        const dist2 = float64_hypot2(distx, disty);

        // compute minimum distance from each other
        const minwidth = this.width + other.width;
        const minheight = this.height + other.height;
        const mindist2 = float64_hypot2(minwidth, minheight);

        // separate when to close
        if (dist2 < mindist2) {
          const dist = float64_sqrt(dist2);
          // compute unit vector normal and tangent vectors
          const unx = +(distx / dist); // unit normal vector x
          const uny = +(disty / dist); // unit normal vector y
          // vec2f_rotn90
          const utx = +(-uny); // unit tangent vector x
          const uty = +(unx); // unit tangent vector y
          // compute scalar projection of velocities
          const svn = float64_dot(unx, uny, this.vx, this.vy);
          const svt = float64_dot(utx, uty, this.vx, this.vy);
          const ovn = float64_dot(unx, uny, other.vx, other.vy);
          // - const ovt = +float64_dot(utx, uty, other.vx, other.vy);
          // compute new velocity using 1 dimension
          const svp = +((svn * (this.mass - other.mass) + 2.0 * other.mass * ovn) / (this.mass + other.mass));
          // - const ovp = +((ovn * (other.mass - this.mass) + 2.0 * this.mass * svn) / (this.mass + other.mass));
          // compute new normal and tangent velocity vectors
          const nnx = +(svp * unx); // nnv = unv * svp
          const nny = +(svp * uny);
          const ntx = +(svt * utx); // ntv = utv * svt;
          const nty = +(svt * uty);
          const nvx = +(nnx + ntx); // nvv = ntv + nnv;
          const nvy = +(nny + nty);
          // compute weights relative to distance
          const reldist = 1.0; // +(1.0 / (+mathf64_abs(distance - mindist) + 1.0)); 
          rule1vx += +(nvx * reldist);
          rule1vy += +(nvy * reldist);
          rule1cnt++;
        }
      }
    }
  }

  paint(ctx, size, properties, args) {
    
  }
}


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

export default function createBoids(viewport = {}, boidCount = 52, maxSize = 254) {

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

      // init flocking rules variable
      let rulesvx = 0.0, rulesvy = 0.0, rulescnt = 0;
      // init separation rule
      let rule1vx = 0.0, rule1vy = 0.0, rule1cnt = 0;

      // clean our canvas and iterate of all boids
      let srcx = 0.0, srcy = 0.0;
      let srcvx = 0.0, srcvy = 0.0;
      let srca = 0.0, srcm = 0.0;
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
        srcm = +(srch * srcw) * 0.639; // mass
        color = 'blue';

        rulesvx = rule1vx = 0.0;
        rulesvy = rule1vy = 0.0; 
        rulescnt = rule1cnt = 0;

        // iterate through other boids
        for (let ioth = 0; ioth < boidCount * structSize; ioth += structSize) {
          if (ioth !== isrc) {
            // load the other boid variables
            const othx = +cboidsf[ioth];
            const othy = +cboidsf[ioth + 1];
            const othvx = +cboidsf[ioth + 2];
            const othvy = +cboidsf[ioth + 3];
            const otha = +cboidsf[ioth + 4];
            const othh = +cboidsf[ioth + 5]; // height/radiusY
            const othw = +(othh / 2.0); // width/radiusX
            const othm = +(othh * othw) * 0.639; // mass

            // compute distance from each other
            const distx = +(othx - srcx);
            const disty = +(othy - srcy);
            const dist2 = +float64_hypot2(distx, disty);

            // compute minimum distance from each other
            const minwidth = +(srcw + othw);
            const minheight = +(srch + othh);
            const mindist2 = +float64_hypot2(minwidth, minheight);

            // define maximum distance for entering branch
            const maxdist = +(+mindist2 + +(mathf64_PI * mathf64_PI));
            if (dist2 < mindist2) {
              const distance = +float64_sqrt(dist2);
              const mindistance = +float64_sqrt(mindist2);
              
              //#region RULE 1: Separation

              // compute unit vector normal and tangent vectors
              const unx = +(distx / distance); // unit normal vector x
              const uny = +(disty / distance); // unit normal vector y
              // vec2f_rotn90
              const utx = +(-uny); // unit tangent vector x
              const uty = +(unx); // unit tangent vector y
              
              // compute scalar projection of velocities
              const svn = +float64_dot(unx, uny, srcvx, srcvy);
              const svt = +float64_dot(utx, uty, srcvx, srcvy);
              const ovn = +float64_dot(unx, uny, othvx, othvy);
              // const ovt = +float64_dot(utx, uty, othvx, othvy);

              // compute new velocity using 1 dimension
              const svp = +((svn * (srcm - othm) + 2.0 * othm * ovn) / (srcm + othm));
              // const ovp = +((ovn * (othm - srcm) + 2.0 * srcm * svn) / (srcm + othm));
              
              // compute new normal and tangent velocity vectors
              const nnx = +(svp * unx); // nnv = unv * svp
              const nny = +(svp * uny);
              const ntx = +(svt * utx); // ntv = utv * svt;
              const nty = +(svt * uty);
              const nvx = +(nnx + ntx); // nvv = ntv + nnv;
              const nvy = +(nny + nty);

              // compute weights relative to distance
              const reldist = 1.0; // +(1.0 / (+mathf64_abs(distance - mindist) + 1.0)); 
              rule1vx += +(nvx * reldist);
              rule1vy += +(nvy * reldist);
              rule1cnt++;

              //#endregion
              
            }
          }
        }

        // compute separation rule
        if (rule1cnt > 0) {
          rulesvx += +(rule1vx / rule1cnt);
          rulesvy += +(rule1vy / rule1cnt);
          ++rulescnt;
        }

        // compute all rules and add it to the source velocity
        if (rulescnt > 0) {
          rulesvx /= rulescnt;
          rulesvy /= rulescnt;
          // srcvx += rulesvx;
          // srcvy += rulesvy;
        }

        // cage the boid to the outer rectangle
        if (false) {
          if (srcvx < 0 && (srcx + srcvx) < srch) {
            srcvx = +mathf64_abs(srcvx);
          }
          else if (srcvx > 0 && (vpwidth - (srcx + srcvx)) < srcw) {
            srcvx = +(-(srcvx));
          }
          if (srcvy < 0 && (srcy + srcvy) < srch) {
            srcvy = +mathf64_abs(srcvy);
          }
          else if (srcvy > 0 && (vpheight - (srcy + srcvy)) < srch) {
            srcvy = +(-(srcvy));
          }
        }
        else {
          const maxx = +(+srch * +(mathf64_PI * mathf64_PI));
          const newx = +(+srcx + +srcvx);
          if (srcvx < 0) {
            const rdistx = +(+maxx - +newx);
            if (+rdistx > 0) {
              const distx = +(maxx - rdistx);
              if (+distx < +srch)
                srcvx = +mathf64_abs(srcvx);
              else {
                color = 'red'
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
            const dsttheta = +mathf64_atan2(dstvy, dstvx);
            const dstmag = +float64_hypot(dstvx, dstvy);

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
                const angle = +mathf64_atan2(ldy, ldx);
                const tx = +(mathf64_cos(angle) * ldmin * 1.0003);
                const ty = +(mathf64_sin(angle) * ldmin * 1.0003);
                const sdx = +(dstx - (srcx + tx));
                const sdy = +(dsty - (srcy + ty));
                const ddx = +(srcx - (dstx + tx));
                const ddy = +(srcy - (dsty + ty));
                const vx = +((dstrad - srcrad) * sdx + (dstrad + dstrad) * ddx) / ldmin;
                const vy = +((dstrad - srcrad) * sdy + (dstrad + dstrad) * ddy) / ldmin;
                const hyp = +float64_hypot(vx, vy);
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
        if (false && rule4cnt > 0) {
          // collision
          rule4vx = rule4vx / rule4cnt ;
          rule4vy = rule4vy / rule4cnt;

          srcvx += rule4vx;
          srcvy += rule4vy;
          srcvx /= 2;
          srcvy /= 2;
        }
        else {
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
        if (true) {
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
        else {
          if (srcx + srcvx < 0) {
            srcx = size.width - (srcx - srcvx);
          }
          else if (srcx + srcvx > size.width) {
            srcx = (srcx + srcvx) - size.width;
          }
          if (srcy + srcvy < 0) {
            srcy = size.height - (srcy - srcvy);
          }
          else if (srcy + srcvy > size.height) {
            srcy = (srcy + srcvy) - size.height;
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

        ctx.save()

        ctx.translate(srcx, srcy)
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
