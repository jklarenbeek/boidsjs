import common from "./common";
import createBoids from "./boids";

function main() {
  const canvas = document.getElementById('boids-canvas');
  const ctx = canvas.getContext('2d');

  const boids = createBoids({ width: canvas.width, height: canvas.height });
  requestAnimationFrame(function draw(now) {
    requestAnimationFrame(draw);
    boids.paint(ctx, { width: canvas.width, height: canvas.height });
  });
}

main();
