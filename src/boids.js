
export class Boids {
}

const CONST_DEFAULT_BOID_RADIUS = 21.5;
const CONST_DEFAULT_SPEED_LIMIT = Math.PI / 3;

export default function createBoids(viewport = {}, boidCount = 112, maxSize = 254) {

  const structSize = 7;
  //const boids = new Int32Array(maxSize * structSize);
  const boidsf = new Float64Array(maxSize * structSize); // boids.buffer);

  // init boids randomly
  for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
    boidsf[isrc] = Math.random() * viewport.width;
    boidsf[isrc + 1] = Math.random() * viewport.height;
    boidsf[isrc + 2] = +Math.sin(Math.random() * Math.PI*2) * CONST_DEFAULT_SPEED_LIMIT;
    boidsf[isrc + 3] = +Math.sin(Math.random() * Math.PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
    boidsf[isrc + 4] = +Math.max(3, Math.abs(Math.sin(Math.random() * Math.PI * 2)) * CONST_DEFAULT_BOID_RADIUS);
    boidsf[isrc + 5] = 0;
    boidsf[isrc + 6] = 0;
  }

  class BoidsImpl {
    paint(ctx, size, properties, args) {
      // the view angle of the boid looking forward.
      const viewAngle = 270 * (Math.PI / 180);

      // index to the other boids
      let idst = 0;

      // indexof and variables for the current boid
      let isrc = 0;
      let srcx = 0.0, srcy = 0.0, srcvx = 0.0, srcvy = 0.0, srcrad = 0.0;

      // aggregated velocity for the separation rule
      let rule1cnt = 0;
      let rule1vx = 0.0, rule1vy = 0.0;

      // aggregated velocity of the alignment rule
      let rule2cnt = 0;
      let rule2vx = 0.0, rule2vy = 0.0;

      // median position of the cohesion rule
      let rule3cnt = 0;
      let rule3x = 0.0, rule3y = 0.0;

      // direct collision velocity.
      let rule4cnt = 0;
      let rule4vx = 0.0, rule4vy = 0.0;

      // aggregated velocities of all rules combined.
      let rulescnt = 0;
      let rulesvx = 0.0;
      let rulesvy = 0.0;

      // clean our canvas and iterate of all boids
      ctx.clearRect(0, 0, size.width, size.height);
      ctx.beginPath();
      for (isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        // load source boid variables from typed array
        srcx = boidsf[isrc]; // x position
        srcy = boidsf[isrc + 1]; // y position
        srcvx = boidsf[isrc + 2]; // speed x-axis
        srcvy = boidsf[isrc + 3]; // speed y-axis
        srcrad = boidsf[isrc + 4]; // radius (TODO: radius-x and radius-y)

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

        // get angle of source boid in radians
        const theta = +Math.atan2(srcvy, srcvx);
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
            const euc2d = Math.sqrt(ldx * ldx + ldy * ldy);
            const lux = ldx / euc2d;
            const luy = ldy / euc2d;

            // we enter when we are at least within some distance.
            if (euc2d < ldmin * 3) {
              
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
                rule4vx += vx;
                rule4vy += vy;
                rule4cnt++;
                continue;
              }

              // view angle detection
              const spdy = (size.height - dsty) - (size.height - srcy);
              const spx = ldx * Math.cos(theta) - spdy * Math.sin(theta);
              const spy = ldx * Math.sin(theta) + spdy * Math.cos(theta);
          
              const spa = Math.atan2(-spy, spx);
          
              const spmin = (-viewAngle) / 2; // -viewingAngle / 2
              const spmax = (+viewAngle) / 2; // +viewingAngle / 2

              // within view? apply flocking rules
              if (spa < spmax && spa > spmin) {
                //const angle = Math.atan2(ldy, ldx);
                //const tx = (Math.cos(angle) * ldmin * 1.0003);
                //const ty = (Math.sin(angle) * ldmin * 1.0003);
                //const sdx = (dstx - (srcx + tx));
                //const sdy = (dsty - (srcy + ty));
                //const ddx = (srcx - (dstx + tx));
                //const ddy = (srcy - (dsty + ty));
                //const vx = ((dstrad - srcrad) * sdx + (dstrad + dstrad) * ddx) / ldmin;
                //const vy = ((dstrad - srcrad) * sdy + (dstrad + dstrad) * ddy) / ldmin;
                const srcmag = Math.sqrt(srcvx * srcvx + srcvy * srcvy);
                // separate
                rule1vx += (srcvx / srcmag + (lux * -1)) / 2;
                rule1vy += (srcvy / srcmag + (luy * -1)) / 2;
                // rule1vx += +((srcvx + (+(srcx - dstx) / +euc2d)) / 2.0);
                // rule1vy += +((srcvy + (+(srcy - dsty) / +euc2d)) / 2.0);
                rule1cnt++;

                // cohesion
                rule3x += dstx;
                rule3y += dsty;
                rule3cnt++;
              }

              // alignment
              // TODO: add weights to its size.
              const dstmag = Math.sqrt(dstvx * dstvx + dstvy * dstvy);
              rule2vx += (dstvx / dstmag);
              rule2vy += (dstvy / dstmag);
              rule2cnt++;
            }
        
            // alignment
            //rule2vx += (dstvx);
            //rule2vy += (dstvy);
            //rule2cnt++;


          }
        }

        // accumulate and balance rules
        if (rule4cnt > 0) {
          // collision
          rule4vx = rule4vx / rule4cnt ;
          rule4vy = rule4vy / rule4cnt;

          srcvx += rule4vx;
          srcvy += rule4vy;
          srcvx /= 2;
          srcvy /= 2;
        }
        else if (true) {
          rulesvx = 0; //srcvx;
          rulesvy = 0; //srcvy;
          rulescnt = 1;
          if (rule1cnt > 0) {
            // separate
            rulesvx += (rule1vx / rule1cnt);
            rulesvy += (rule1vy / rule1cnt);
            rulescnt++;
          }
          if (rule2cnt > 0) {
            // alignment
            rulesvx += (rule2vx / rule2cnt);
            rulesvy += (rule2vy / rule2cnt);
            rulescnt++;
          }
          if (rule3cnt > 0) {
            // cohesion
            const vx = ((rule3x / rule3cnt) - srcx);
            const vy = ((rule3y / rule3cnt) - srcy);
            const cm = Math.sqrt(vx * vx + vy * vy);
            rulesvx += (srcvx + (vx / (cm))) / Math.PI;
            rulesvy += (srcvy + (vy / (cm))) / Math.PI;
            rulescnt++;
          }
          if (rulescnt > 0) {
            rulesvx /= rulescnt;
            rulesvy /= rulescnt;
          }

          srcvx += (rulesvx); // / (Math.PI * Math.PI));
          srcvy += (rulesvy); // / (Math.PI * Math.PI));
        }


        // limit speed of boid
        const srcmag = Math.sqrt(srcvx * srcvx + srcvy * srcvy);
        if (srcmag > CONST_DEFAULT_SPEED_LIMIT) {
          srcvx = (srcvx / srcmag) * CONST_DEFAULT_SPEED_LIMIT;
          srcvy = (srcvy / srcmag) * CONST_DEFAULT_SPEED_LIMIT;
        }

        // cage boid to outer rectangle
        if (false) {
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

        srcx += srcvx;
        srcy += srcvy;

        boidsf[isrc] = srcx;
        boidsf[isrc + 1] = srcy;
        boidsf[isrc + 2] = srcvx;
        boidsf[isrc + 3] = srcvy;
        boidsf[isrc + 4] = srcrad;

        if (rule1cnt > 0) ctx.fillStyle = `rgb(${127 + parseInt(rulesvx)}, ${127 + parseInt(rulesvy)}, 0)`;
        else ctx.fillStyle = 'blue';

        ctx.save()

        ctx.translate(srcx, srcy)
        ctx.rotate(theta)
  
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
        ctx.arc(0, 0, srcrad, 0, 2 * Math.PI, false);
        ctx.fill();
        //ctx.moveTo(srcx, srcy);
        //ctx.lineTo(rule1vx, rule1vy);
        //ctx.stroke();
        // ctx.moveTo(srcx, srcy);
        //ctx.lineTo(srcx + rule1vx, srcy + rule1vy);

        // ctx.closePath();

        ctx.restore();

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
