

export const fp_default = new Uint32Array(4);

export const fp_size = 32;
export const fp_bits = 4;
export const fp_zero = 256;

export function setZeroA(isrc = 0, src = fp_default) {
    src[isrc] = this.zero;
    src[isrc + 1] = this.zero;
}
export function setPositionA(isrc = 0, src = fp_default, x = 0, y = 0) {
    src[isrc] = (((x|0) + fp_zero)|0) << fp_bits;
    src[isrc + 1] = (((y|0) + fp_zero)|0) << fp_bits;
}
export function setVelocityA(isrc = 0, src = fp_default, vx = 0, vy = 0) {
  src[isrc + 2] = (((vx|0) + fp_zero)|0) << fp_bits;
  src[isrc + 3] = (((vy|0) + fp_zero)|0) << fp_bits;
}

export function getMagnitudeA(isrc = 0, src = fp_default) {
  const vy = ((src[isrc + 2] | 0) - fp_zero) >> fp_bits;
  const vy = ((src[isrc + 3] | 0) - fp_zero) >> fp_bits;
  return Math.sqrt(vx * vx + vy * vy);
}

export function getDistanceA(isrc = 0, src, idst = 0, dst) {
  const dx = (((dst[idst + 2] | 0) - (src[isrc + 2] | 0) | 0) - fp_zero);
  const dy = (((dst[idst + 3] | 0) - (src[isrc + 3] | 0) | 0) - fp_zero);
  return Math.sqrt(dx * dx + dy * dy);
}