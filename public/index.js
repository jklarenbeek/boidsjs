const int_MULTIPLIER = 10000;

//#region trigonometry

const int_PI = (Math.PI * int_MULTIPLIER)|0;
const int_PI_A = ((4 / Math.PI) * int_MULTIPLIER)|0;
const int_PI_B = ((4 / (Math.PI * Math.PI)) * int_MULTIPLIER)|0;

const def_vec2i = Object.seal({ x: 0, y: 0 });
const def_vec2f = Object.seal({ x: 0.0, y: 0.0 });
const def_vec3f = Object.seal({ x: 0.0, y: 0.0, z: 0.0 });

function float_sqrt(n = 0.0) {
  n = +n;
  return +Math.sqrt(+n);
}

function float_hypot2(dx = 0.0, dy = 0.0) {
  return +(+(+dx * +dx) + +(+dy * +dy));
}

/**
 * 
 * We can calculate the Dot Product of two vectors this way:
 * 
 *    a · b = |a| × |b| × cos(θ)
 * 
 * or in this implementation as:
 * 
 *    a · b = ax × bx + ay × by
 * 
 * When two vectors are at right angles to each other the dot product is zero.
 * 
 * @param {float} ax vector A x velocity 
 * @param {float} ay vector A y velocity
 * @param {float} bx vector B x velocity
 * @param {float} by vector B y velocity
 * @returns {float} scalar of the dot product
 */
function float_dot(ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0) {
  return +(+(+ax * +bx) + +(+ay * +by));
}

//#region trigonometry

const float_PIx2 = Math.PI * 2; // 6.28318531
const float_PIh = Math.PI / 2; // 1.57079632
const float_PI_A = 4 / Math.PI; // 1.27323954
const float_PI_B = 4 / (Math.PI * Math.PI); // 0.405284735

const CONST_DEFAULT_BOID_RADIUS = 21.5;
const CONST_DEFAULT_SPEED_LIMIT = Math.PI / 3;

function initBoidsf(boidsf, count, size, viewport) {

  // init boids randomly
  for (let isrc = 0; isrc < count * size; isrc += size) {
    // x-position
    boidsf[isrc] = Math.random() * viewport.width; // srcx
    // y-position
    boidsf[isrc + 1] = Math.random() * viewport.height; // srcy
    // x-velocity
    boidsf[isrc + 2] = +Math.sin(Math.random() * Math.PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
    // y-velocity
    boidsf[isrc + 3] = +Math.sin(Math.random() * Math.PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
    // angle in unsigned radians
    boidsf[isrc + 4] = (Math.random() * Math.PI * 2) - Math.PI;
    // unsigned radiusX or width
    boidsf[isrc + 5] = +Math.max(3, Math.abs(Math.sin((Math.random() * Math.PI * 2) - Math.PI)) * CONST_DEFAULT_BOID_RADIUS);
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
      // rollup optimization strangness fix
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

      for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        srcx = +cboidsf[isrc]; // x position
        srcy = +cboidsf[isrc + 1]; // y position
        srcvx = +cboidsf[isrc + 2]; // x velocity
        srcvy = +cboidsf[isrc + 3]; // y velocity
        srca = +cboidsf[isrc + 4]; // angle (derived from vx/vy when mag > 0.0)
        srch = +cboidsf[isrc + 5]; // height/radiusY
        srcw = +(srch / 2.0);  // width/radiusX
        srcm = +(srch * srcw) * 0.639; // mass

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
            const othh = +cboidsf[ioth + 5]; // height/radiusY
            const othw = +(othh / 2.0); // width/radiusX
            const othm = +(othh * othw) * 0.639; // mass

            // compute distance from each other
            const distx = +(othx - srcx);
            const disty = +(othy - srcy);
            const dist2 = +float_hypot2(distx, disty);

            // compute minimum distance from each other
            const minwidth = +(srcw + othw);
            const minheight = +(srch + othh);
            const mindist2 = +float_hypot2(minwidth, minheight);

            // define maximum distance for entering branch
            const maxdist = +(mindist2 * Math.PI);
            if (dist2 < mindist2) {
              const distance = +float_sqrt(dist2);
              const mindistance = +float_sqrt(mindist2);
              
              //#region RULE 1: Separation

              // compute unit vector normal and tangent vectors
              const unx = +(distx / distance); // unit normal vector x
              const uny = +(disty / distance); // unit normal vector y
              // vec2f_rotn90
              const utx = +(-uny); // unit tangent vector x
              const uty = +(unx); // unit tangent vector y
              
              // compute scalar projection of velocities
              const svn = +float_dot(unx, uny, srcvx, srcvy);
              const svt = +float_dot(utx, uty, srcvx, srcvy);
              const ovn = +float_dot(unx, uny, othvx, othvy);
              // const ovt = +float_dot(utx, uty, othvx, othvy);

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
              const reldist = 1.0; // +(1.0 / (+Math.abs(distance - mindist) + 1.0)); 
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
          srcvx += rulesvx;
          srcvy += rulesvy;
        }

        // cage the boid to the outer rectangle
        {
          if (srcvx < 0 && (srcx + srcvx) < srch) {
            srcvx = +Math.abs(srcvx);
          }
          else if (srcvx > 0 && (options.width - (srcx + srcvx)) < srcw) {
            srcvx = +(-(srcvx));
          }
          if (srcvy < 0 && (srcy + srcvy) < srch) {
            srcvy = +Math.abs(srcvy);
          }
          else if (srcvy > 0 && (options.height - (srcy + srcvy)) < srch) {
            srcvy = +(-(srcvy));
          }
        }

        //const srcvx = Math.cos(newangle) * newmag;
        //const srcvy = Math.sin(newangle) * newmag;

        srcx += +srcvx;
        srcy += +srcvy;

        // save boid state
        nboidsf[isrc] = srcx;
        nboidsf[isrc + 1] = srcy;
        nboidsf[isrc + 2] = srcvx;
        nboidsf[isrc + 3] = srcvy;
        nboidsf[isrc + 4] = srca;
        nboidsf[isrc + 5] = srch;

        // draw the boid
        callback(srcx, srcy, srch, srcw, +Math.atan(srcvx, srcvy));
      }

      // we are done processing
      // flip buffers (something funny with rollup...)
      isBuffer1 = buf1 ? false : true;

    }
    paint2(ctx, size, properties, args) {
      // the view angle of the boid looking forward.
      const viewAngle = 270 * (Math.PI / 180);
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
        srcrad = boidsf[isrc + 4]; // radius (TODO: radius-x and radius-y)
        // get angle of source boid in radians
        const srctheta = +Math.atan2(srcvy, srcvx);
        const srcmag = +float_hypot(srcvx, srcvy);

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
            const dstrad = boidsf[idst + 4];
            const dstmag = +float_hypot(dstvx, dstvy);

            // calculate basic distance
            const ldmin = srcrad + dstrad;
            const ldx = dstx - srcx;
            const ldy = dsty - srcy;
            const euc2d = +float_hypot(ldx, ldy);
            const lux = ldx / euc2d;
            const luy = ldy / euc2d;

            // we enter when we are at least within some distance.
            if (euc2d < ldmin * 2) {
              
              // collision detection
              if (euc2d < ldmin) { // TODO: mass and velocity is not correctly transfered.
                const angle = Math.atan2(ldy, ldx);
                const tx = (Math.cos(angle) * ldmin * 1.0003);
                const ty = (Math.sin(angle) * ldmin * 1.0003);
                const sdx = (dstx - (srcx + tx));
                const sdy = (dsty - (srcy + ty));
                const ddx = (srcx - (dstx + tx));
                const ddy = (srcy - (dsty + ty));
                const vx = ((dstrad - srcrad) * sdx + (dstrad + dstrad) * ddx) / ldmin;
                const vy = ((dstrad - srcrad) * sdy + (dstrad + dstrad) * ddy) / ldmin;
                const hyp = float_hypot(vx, vy);
                //rule4vx += (vx / hyp) / Math.PI;
                //rule4vy += (vy / hyp) / Math.PI;
                rule4vx += (lux * -1) * 0.853;
                rule4vy += (luy * -1) * 0.853;
                rule4cnt++;
                //continue;
              }

              // view angle detection
              const spdy = (size.height - dsty) - (size.height - srcy);
              const spx = ldx * Math.cos(srctheta) - spdy * Math.sin(srctheta);
              const spy = ldx * Math.sin(srctheta) + spdy * Math.cos(srctheta);
          
              const spa = Math.atan2(-spy, spx);
          
              // within view? apply flocking rules
              if (spa < maxViewAngle && spa > minViewAngle) {
                //const angle = Math.atan2(ldy, ldx);
                //const tx = (Math.cos(angle) * ldmin * 1.0003);
                //const ty = (Math.sin(angle) * ldmin * 1.0003);
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
              const dstmag = float_hypot(dstvx, dstvy);
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
            const nm = float_hypot(vx, vy);
            rulesvx += (srcvx + (vx / nm)) / Math.PI;
            rulesvy += (srcvy + (vy / nm)) / Math.PI;
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
            srcvx += (rulesvx);// * 0.03; // / (Math.PI * Math.PI));
            srcvy += (rulesvy);// * 0.03; // / (Math.PI * Math.PI));
          }
        }

        //#endregion

        //#region limit source boid

        // limit speed of boid
        const newmag = +float_hypot(srcvx, srcvy);
        if (newmag > CONST_DEFAULT_SPEED_LIMIT) {
          srcvx = (srcvx / newmag) * CONST_DEFAULT_SPEED_LIMIT;
          srcvy = (srcvy / newmag) * CONST_DEFAULT_SPEED_LIMIT;
        }

        // cage boid to outer rectangle
        {
          if (srcvx < 0 && (srcx + srcvx) < srcrad) {
            srcvx = Math.abs(srcvx);
          }
          else if (srcvx > 0 && (size.width - (srcx + srcvx)) < srcrad) {
            srcvx = -(srcvx);
          }
          if (srcvy < 0 && (srcy + srcvy) < srcrad) {
            srcvy = Math.abs(srcvy);
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
        boidsf[isrc + 4] = srcrad;

        //#region draw boid
        if (rule4cnt > 0) ctx.fillStyle = 'red';
        else if (rule1cnt > 0) ctx.fillStyle = `green`;
        else if (rule2cnt > 0) ctx.fillStyle = `yellow`;
        else if (rule3cnt > 0) ctx.fillStyle = `purple`;
        else ctx.fillStyle = 'blue';

        ctx.save();

        ctx.translate(srcx, srcy);
        ctx.rotate(Math.atan2(srcvy, srcvx));
  
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
        //ctx.arc(0, 0, srcrad, 0, 2 * Math.PI, false);
        ctx.ellipse(0, 0, srcrad, srcrad / 2, 0, 0, Math.PI * 2);
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
    {

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.beginPath();

      boids.processFrame(
        { width: canvas.clientWidth, height: canvas.clientHeight },
        function draw(x = 0.0, y = 0.0, w = 0.0, h = 0.0, a = 0.0) {
          ctx.save();
          ctx.translate(x, y);
          ctx.rotate(a);
          ctx.beginPath();
          ctx.fillStyle = 'blue';
          ctx.ellipse(0, 0, w, h, 0, 0, Math.PI * 2);
          ctx.fill();
          ctx.restore();
        }
      );
    }
  });
}

main();
//# sourceMappingURL=index.js.map
