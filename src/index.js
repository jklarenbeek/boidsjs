import createBoids from "./boids";

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
