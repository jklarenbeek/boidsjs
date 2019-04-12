import createBoids from "./boids";

function main() {
  const canvas = document.getElementById('boids-canvas');
  const ctx = canvas.getContext('2d');

  const boids = createBoids({ width: canvas.clientWidth, height: canvas.clientHeight });
  requestAnimationFrame(function draw(now) {
    requestAnimationFrame(draw);
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    if (false) {

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.beginPath();

      boids.processFrame(
        { width: canvas.clientWidth, height: canvas.clientHeight },
        function drawBoid(x = 0.0, y = 0.0, w = 0.0, h = 0.0, a = 0.0, color = 'blue') {
          ctx.save();
          ctx.translate(x, y);
          ctx.rotate(a);
          ctx.beginPath();
          ctx.fillStyle = color;
          ctx.ellipse(0, 0, w, h, 0, 0, Math.PI * 2);
          ctx.fill();
          ctx.restore();
        }
      );
    }
    else
      boids.paint(ctx, { width: canvas.clientWidth, height: canvas.clientHeight });
  });
}

main();
