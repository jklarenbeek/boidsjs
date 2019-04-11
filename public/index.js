const int_MULTIPLIER = 10000;

//#region trigonometry

const int_PI = (Math.PI * int_MULTIPLIER)|0;
const int_PI_A = ((4 / Math.PI) * int_MULTIPLIER)|0;
const int_PI_B = ((4 / (Math.PI * Math.PI)) * int_MULTIPLIER)|0;

const def_vec2i = Object.seal({ x: 0, y: 0 });
const def_vec2f = Object.seal({ x: 0.0, y: 0.0 });
const def_vec3f = Object.seal({ x: 0.0, y: 0.0, z: 0.0 });

function float_hypot(dx = 0.0, dy = 0.0) {
  return +Math.sqrt(+(+(+dx * +dx) + +(+dy * +dy)));
}

//#region trigonometry

const float_PIx2 = Math.PI * 2; // 6.28318531
const float_PIh = Math.PI / 2; // 1.57079632
const float_PI_A = 4 / Math.PI; // 1.27323954
const float_PI_B = 4 / (Math.PI * Math.PI); // 0.405284735

const CONST_DEFAULT_SPEED_LIMIT = Math.PI / 3;

function createBoids(viewport = {}, boidCount = 52, maxSize = 254) {

  const structSize = 7;
  //const boids = new Int32Array(maxSize * structSize);
  const boidsfBuffer1 = new Float64Array(maxSize * structSize); // boids.buffer);

  class BoidsImpl {
    paint(ctx, size, properties, args) {
      // get current buffer
      const boidsf = boidsfBuffer1;

      // clean our canvas and iterate of all boids
      ctx.clearRect(0, 0, size.width, size.height);
      ctx.beginPath();
      for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        const srcx = +boidsf[isrc]; // x position
        const srcy = +boidsf[isrc + 1]; // y position
        const srcvx = +boidsf[isrc + 2]; // x velocity
        const srcvy = +boidsf[isrc + 3]; // y velocity
        const srcangle = +boidsf[isrc + 4]; // angle (derived from vx/vy when mag > 0.0)
        const srch = +boidsf[isrc + 5]; // height/radiusY
        const srcw = +(srch / 2.0);  // width/radiusX

        // iterate through other boids
        for (let ioth = 0; ioth < boidCount * structSize; ioth += structSize) {
          if (ioth !== isrc) {
            // load the other boid variables
            const othx = +boidsf[ioth];
            const othy = +boidsf[ioth + 1];
            const othvx = +boidsf[ioth + 2];
            const othvy = +boidsf[ioth + 3];
            const othh = +boidsf[ioth + 5]; // height/radiusY
            const othw = +(othh / 2.0); // width/radiusX

            const distx = +(othx - srcx);
            const disty = +(othy - srcy);
            const distance = +float_hypot(distx, disty);

            const minwidth = +(srcw + othw);
            const minheight = +(srch + othh);
            const mindist = +float_hypot(minwidth, minheight);
            const maxdist = +(mindist * Math.PI);
            if (distance < maxdist) {
              
              //#region RULE 1: Separation

              // compute unit normal and tangent vectors
              const unx = +(distx / distance); // unit normal vector x
              const uny = +(disty / distance); // unit normal vector y
              const utx = +(-uny); // unit tangent vector x
              const uty = +(unx); // unit tangent vector y
              
              // compute scalar projection of velocities
              const svn = +Float_dot2x2(unx, uny, srcvx, srcvy);
              const svt = +Float_dot2x2(utx, uty, srcvx, srcvy);
              const ovn = +Float_dot2x2(unx, uny, othvx, othvy);

              //#endregion
              
            }
          }
        }

        let newangle = srcangle;
        let newmag = srcmag;

        //const srcvx = Math.cos(newangle) * newmag;
        //const srcvy = Math.sin(newangle) * newmag;
        const newx = srcx + srcvx;
        const newy = srcy + srcvy;

        // save boid state
        boidsf[isrc] = newx;
        boidsf[isrc + 1] = newy;
        boidsf[isrc + 2] = newangle;
        boidsf[isrc + 3] = newmag;

        // draw the boid
        ctx.save();
        ctx.translate(newx, newy);
        ctx.rotate(newangle);
        ctx.beginPath();
        ctx.fillStyle = 'blue';
        ctx.ellipse(0, 0, srcradw, srcradh, 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }
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
    boids.paint(ctx, { width: canvas.clientWidth, height: canvas.clientHeight });
  });
}

main();
