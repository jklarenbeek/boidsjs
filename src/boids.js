
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
      
      ctx.clearRect(0, 0, size.width, size.height);
      ctx.beginPath();
      for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        // load variables from array
        let srcx = boidsf[isrc]; // x position
        let srcy = boidsf[isrc + 1]; // y position
        let srcvx = boidsf[isrc + 2]; // speed x-axis
        let srcvy = boidsf[isrc + 3]; // speed y-axis
        let srcrad = boidsf[isrc + 4]; // radius (TODO: radius-x and radius-y)
        const srcvwangle = 270 * (Math.PI / 180);
        boidsf[isrc + 5] = srcx;
        boidsf[isrc + 6] = srcy;
        // let srcsqn = srcvx * srcvx + srcvy * srcvy;
        // let srcmagnitude = Math.pow(srcsqn, .5);
        // let srcangle = Math.atan2(srcvy, srcvy);
        const theta = +Math.atan2(srcvy, srcvx);

        // separate
        let rule1vx = 0.0;
        let rule1vy = 0.0;
        let rule1cnt = 0;

        // alignment
        let rule2vx = 0.0;
        let rule2vy = 0.0;
        let rule2cnt = 0;
        
        // cohesion
        let rule3x = 0.0;
        let rule3y = 0.0;
        let rule3cnt = 0;
        
        // collision detection
        let rule4vx = 0.0;
        let rule4vy = 0.0;
        let rule4cnt = 0;

        // aggrates
        let accx = 0.0;
        let accy = 0.0;
        let accn = 0;

        for (let idst = 0; idst < boidCount * structSize; idst += structSize) {
          if (idst !== isrc) {
            const dstx = boidsf[idst];
            const dsty = boidsf[idst + 1];
            const dstvx = boidsf[idst + 2];
            const dstvy = boidsf[idst + 3];
            const dstrad = boidsf[idst + 4];

            const ldmin = srcrad + dstrad;
            const ldx = dstx - srcx;
            const ldy = dsty - srcy;
            const euc2d = Math.sqrt(ldx * ldx + ldy * ldy);

            // we enter when we are at least within some distance.
            if (euc2d < CONST_DEFAULT_BOID_RADIUS * 5) {
              
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
          
              const spmin = (-srcvwangle) / 2; // -viewingAngle / 2
              const spmax = (+srcvwangle) / 2; // +viewingAngle / 2

              // within view? apply flocking rules
              if (spa < spmax && spa > spmin) {
                const angle = Math.atan2(ldy, ldx);
                const tx = (Math.cos(angle) * ldmin * 1.0003);
                const ty = (Math.sin(angle) * ldmin * 1.0003);
                const sdx = (dstx - (srcx + tx));
                const sdy = (dsty - (srcy + ty));
                const ddx = (srcx - (dstx + tx));
                const ddy = (srcy - (dsty + ty));
                const vx = ((dstrad - srcrad) * sdx + (dstrad + dstrad) * ddx) / ldmin;
                const vy = ((dstrad - srcrad) * sdy + (dstrad + dstrad) * ddy) / ldmin;

                // separate
                rule1vx += vx;
                rule1vy += vy;
                rule1cnt++;

                // cohesion
                rule3x += dstx;
                rule3y += dsty;
                rule3cnt++;
              }

            // alignment
            // TODO: add weights to its size.
            rule2vx += (dstvx);
            rule2vy += (dstvy);
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
          accx = 0; //srcvx;
          accy = 0; //srcvy;
          accn = 1;
          if (rule1cnt > 0) {
            // separate
            accx += (rule1vx / rule1cnt) / (Math.PI * 1.47);
            accy += (rule1vy / rule1cnt) / (Math.PI * 1.47);
            accn++;
          }
          if (rule2cnt > 0) {
            // alignment
            accx += (rule2vx / rule2cnt) * Math.PI;
            accy += (rule2vy / rule2cnt) * Math.PI;
            accn++;
          }
          if (rule3cnt > 0) {
            // cohesion
            accx += ((rule3x / rule3cnt) - srcx) / (Math.PI * 1.12);
            accy += ((rule3y / rule3cnt) - srcy) / (Math.PI * 1.12);
            accn++;
          }
          accx /= (accn * 112.3);
          accy /= (accn * 112.3);
          srcvx += accx;
          srcvy += accy;
          //srcvx /= 2;
          //srcvy /= 2;
        }


        // limit speed of boid
        const srcmag = Math.sqrt(srcvx * srcvx + srcvy * srcvy);
        if (srcmag > CONST_DEFAULT_SPEED_LIMIT) {
          srcvx = (srcvx / srcmag) * CONST_DEFAULT_SPEED_LIMIT;
          srcvy = (srcvy / srcmag) * CONST_DEFAULT_SPEED_LIMIT;
        }

        // cage boid to outer rectangle
        if (true) {
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

        if (rule1cnt > 0) ctx.fillStyle = `rgb(${127 + parseInt(accx)}, ${127 + parseInt(accy)}, 0)`;
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
