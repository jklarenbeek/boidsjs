
export const vec_default = new Int32Array(6)
export const vec2_default = new Int32Array(vec_default.buffer, 0, 2);
export const vec4_default = new Int32Array(vec_default.buffer, 0, 4);
export const vec6_default = new Int32Array(vec_default.buffer, 0, 6);
export const fvec2_default = new Float32Array(vec_default.buffer, 0, 2);
export const fvec4_default = new Float32Array(vec_default.buffer, 0, 4);
export const fvec6_default = new Float32Array(vec_default.buffer, 0, 6);

export function loadVec2(isrc, src, dst) {
  dst[0] = src[isrc];
  isrc++;
  dst[1] = src[isrc];
  return dst;
}
export function storeVec2(idst, dst, src) {
  dst[idst] = src[0];
  idst++;
  dst[idst] = src[1]
}

export function vec2_addVec2(left, right) {
  left[0] += right[0];
  left[1] += right[1];
  return left;
}
export function vec2_addXY(left, x, y) {
  left[0] += x;
  left[1] += y;
  return left;
}

export function vec2_subVec2(left, right) {
  left[0] -= right[0];
  left[1] -= right[1];
  return left;
}
export function vec2_subXY(left, x, y) {
  left[0] -= x;
  left[0] -= y;
  return left;
}

export function vec2_mulVec2(left, right) {
  left[0] *= right[0];
  left[1] *= right[1];
  return left;
}

export function vec2_mulScalar(left, right) {
  left[0] *= right;
  left[1] *= right;
  return left;
}

export function vec2_divVec2(left, right) {
  left[0] /= right[0];
  left[1] /= right[1];
  return left;
}
export function vec2_divScalar(left, right) {
  left[0] /= right;
  left[1] /= right;
  return left;
}

export function vec2_dot(left, right) {
  return (left[0] * right[0]) + (left[1] * right[1]);
}
export function vec2_cross(left, right) {
  return (left[0] * right[1]) + (left[1] * right[0]);
}

export function vec2_hypotenuse(left) {
  return Math.pow((left[0]*left[0]) + (left[1]*left[1]), .5);
  // return Math.sqrt(left[0]*left[0] + left[1]*left[1]);
}