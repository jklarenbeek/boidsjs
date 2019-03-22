
export class Boids {
}

const CONST_DEFAULT_BOID_RADIUS = 20;
const CONST_DEFAULT_SPEED_LIMIT = 2;

class Boid {
  constructor(size) {
    this.x = Math.random() * viewport.width;
    this.y = Math.random() * viewport.height;
    this.vx = +Math.sin(Math.random() * Math.PI*2) * CONST_DEFAULT_SPEED_LIMIT;
    this.vy = +Math.sin(Math.random() * Math.PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
    this.radius = CONST_DEFAULT_BOID_RADIUS;
  }
}
export default function createBoids(viewport = {}, boidCount = 3, maxSize = 254) {

  const structSize = 7;
  const boids = new Int32Array(maxSize * structSize);
  const boidsf = new Float32Array(boids.buffer);

  // init boids randomly
  for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
    boids[isrc] = Math.random() * viewport.width;
    boids[isrc + 1] = Math.random() * viewport.height;
    boidsf[isrc + 2] = +Math.sin(Math.random() * Math.PI*2) * CONST_DEFAULT_SPEED_LIMIT;
    boidsf[isrc + 3] = +Math.sin(Math.random() * Math.PI * 2) * CONST_DEFAULT_SPEED_LIMIT;
    boids[isrc + 4] = CONST_DEFAULT_BOID_RADIUS; // Math.max(3, Math.abs(Math.sin(Math.random() * Math.PI * 2)) * CONST_DEFAULT_SPEED_LIMIT);
    boids[isrc + 5] = 0;
    boids[isrc + 6] = 0;
  }

  class BoidsImpl {
    paint(ctx, size, properties, args) {
      
      ctx.clearRect(0, 0, size.width, size.height);
      ctx.beginPath();
      for (let isrc = 0; isrc < boidCount * structSize; isrc += structSize) {
        // load variables from array
        let srcx = boids[isrc] | 0; // x position
        let srcy = boids[isrc + 1]|0; // y position
        let srcvx = boidsf[isrc + 2]; // speed x-axis
        let srcvy = boidsf[isrc + 3]; // speed y-axis
        let srcrad = boids[isrc + 4] | 0; // radius (TODO: radius-x and radius-y)
        const srcvwangle = 120 * (Math.PI / 180);
        boids[isrc + 5] = srcx;
        boids[isrc + 6] = srcy;
        // let srcsqn = srcvx * srcvx + srcvy * srcvy;
        // let srcmagnitude = Math.pow(srcsqn, .5);
        // let srcangle = Math.atan2(srcvy, srcvy);
        const theta = Math.atan2(srcvy, srcvx);

        // separate
        let rule1vx = 0;
        let rule1vy = 0;
        let rule1cnt = 0;

        // alignment
        let rule2vx = 0;
        let rule2vy = 0;

        // cohesion
        let rule3 = 0;
        
        // collision detection
        let rule4vx = srcvx;
        let rule4vy = srcvy;
        let rule4cnt = 1;

        for (let idst = 0; idst < boidCount * structSize; idst += structSize) {
          if (idst !== isrc) {
            const dstx = boids[idst] | 0;
            const dsty = boids[idst + 1] | 0;
            const dstvx = boidsf[idst + 2];
            const dstvy = boidsf[idst + 3];
            const dstrad = boids[idst + 4] | 0;

            const ldmin = srcrad + dstrad;
            const ldx = dstx - srcx;
            const ldy = dsty - srcy;
            const euc2d = Math.sqrt(ldx * ldx + ldy * ldy);

            // separation
            const spdy = (size.height - dsty) - (size.height - srcy);
            const spx = ldx * Math.cos(theta) - spdy * Math.sin(theta);
            const spy = ldx * Math.sin(theta) + spdy * Math.cos(theta);
        
            const spa = Math.atan2(-spy, spx);
        
            const spmin = -srcvwangle / 2; // -viewingAngle / 2
            const spmax = +srcvwangle / 2; // +viewingAngle / 2
            if (spa < spmax && spa > spmin) {
              const spux = ldx / (euc2d);
              const spuy = ldy / (euc2d);
              rule1vx += spux;
              rule1vy += spuy;
              rule1cnt++;
            }
        
            // alignment
            // rule2vx += dstvx;
            // rule2vy += dstvy;

            // collision detection
            if (euc2d < ldmin) {
              const angle = Math.atan2(ldy, ldx);
              const tx = (Math.cos(angle) * ldmin * 1.32);
              const ty = (Math.sin(angle) * ldmin * 1.32);
              const ax = (dstx - (srcx + tx));
              const ay = (dsty - (srcy + ty));
              rule4vx += ax;
              rule4vy += ay;
              rule4cnt++;              
            }

          }
        }

        // separate
        rule1vx = rule1vx / rule1cnt;
        rule1vy = rule1vy / rule1cnt;

        // align
        rule2vx = rule2vx / (boidCount - 1);
        rule2vy = rule2vy / (boidCount - 1);
        rule2vx = (rule2vx) / 5;
        rule2vy = (rule2vy) / 5;

        // collision
        rule4vx = rule4vx / rule4cnt;
        rule4vy = rule4vy / rule4cnt;

        // accumulate and balance rules
        // srcvx += rule1vx;
        // srcvy += rule1vy;
        // srcvx += rule4vx;
        // srcvy += rule4vy;
        //srcvx /= 2;
        //srcvy /= 2;

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
          else if (srcvx > 0 && (viewport.width - (srcx + srcvx)) < srcrad) {
            srcvx = -(srcvx);
          }
          if (srcvy < 0 && (srcy + srcvy) < srcrad) {
            srcvy = Math.abs(srcvy);
          }
          else if (srcvy > 0 && (viewport.height - (srcy + srcvy)) < srcrad) {
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

        boids[isrc] = srcx;
        boids[isrc + 1] = srcy;
        boidsf[isrc + 2] = srcvx;
        boidsf[isrc + 3] = srcvy;
        boids[isrc + 4] = srcrad;

        if (rule1cnt > 0) ctx.fillStyle = `rgb(${127 + parseInt(rule1vx)}, ${127 + parseInt(rule1vy)}, 0)`;
        else ctx.fillStyle = 'blue';

        ctx.save()

        ctx.translate(srcx, srcy)
        ctx.rotate(theta)
  
        ctx.beginPath();

        const boidlength = CONST_DEFAULT_BOID_RADIUS*3.45;
        const bdi = boidlength/72
        ctx.lineTo(-boidlength/4, bdi)
        ctx.lineTo(-boidlength/3, bdi)
        ctx.lineTo(-boidlength/2, boidlength/6)
        ctx.lineTo(-boidlength/2, -boidlength/6)
        ctx.lineTo(-boidlength/3, -bdi)
        ctx.lineTo(-boidlength / 4, -bdi)
        
        ctx.closePath();
        ctx.fill();

        ctx.restore();

        //ctx.moveTo(srcx, srcy);
        //ctx.arc(srcx, srcy, srcrad, 0, 2 * Math.PI, false);
        //ctx.fill();
        //ctx.moveTo(srcx, srcy);
        //ctx.lineTo(rule1vx, rule1vy);
        // ctx.moveTo(srcx, srcy);
        //ctx.lineTo(srcx + rule1vx, srcy + rule1vy);
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
