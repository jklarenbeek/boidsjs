
const fastSin_B = 1.2732395; // 4/pi
const fastSin_C = -0.40528473; // -4 / (pi²)
export function fastSin(value) {
  // See  for graph and equations
  // https://www.desmos.com/calculator/8nkxlrmp7a
  // logic explained here : http://devmaster.net/posts/9648/fast-and-accurate-sine-cosine			
      
  return (value > 0)
    ? fastSin_B * value - fastSin_C * value * value
    : fastSin_B * value + fastSin_C * value * value;
}

export function fastSin2(a) {
  let b, c;
  return a *= 5214
    , c = a << 17
    , a -= 8192
    , a <<= 18
    , a >>= 18
    , a = a * a >> 12
    , b = 19900 - (3516 * a >> 14)
    , b = 4096 - (a * b >> 16)
    , 0 > c && (b = -b)
    , 2.44E-4 * b;
};
  
export function fastSin3(a) {
  a *= 5214;
  let b = a << 17;
  a = a - 8192 << 18 >> 18;
  a = a * a >> 12;
  a = 4096 - (a * (19900 - (3516 * a >> 14)) >> 16);
  0 > b && (a = -a);
  return 2.44E-4 * a
};

export function distance2(dx = 70, dy = 90) {
  const a = Math.atan2(dy, dx);
  const ux = 10 * Math.cos(a);
  const uy = 10 * Math.sin(a);
  return [ux, uy];
}
    
export function distance1(dx = 70, dy = 90) {
  const r = Math.sqrt(dx * dx + dy * dy);
  const ux = 10 * dx / r;
  const uy = 10 * dy / r;
  return [ux, uy];
}

const fastInvSqrt_typed_y = new Float32Array(1);
const fastInvSqrt_typed_i = new Int32Array(fastInvSqrt_typed_y.buffer);
export function fastInvSqrt_typed(n) {
  const n2 = n * 0.5;
  fastInvSqrt_typed_y[0] = n;
  fastInvSqrt_typed_i[0] = 0x5f375a86 - (fastInvSqrt_typed_i[0] >> 1);
  n = fastInvSqrt_typed_y[0];
  return n * (1.5 - (n2 * n * n));
};

export function fastSqrt(value) {
  return Math.pow(value, .5);
}

const fibionacci_sqrtFive = Math.sqrt(5);
export function fibonacci(value) {
  let firstHalf = 1 / fibionacci_sqrtFive * Math.pow( ( ( 1 + fibionacci_sqrtFive ) / 2), value);
  let secondHalf = 1 / fibionacci_sqrtFive * Math.pow( ( (1 - fibionacci_sqrtFive ) / 2 ), value);
  return Math.round(firstHalf - secondHalf);
}

Math.clampInt = Math.clampInt || function Math_clampInt(value, min, max) {
  return Math.min(max, Math.max(min, value));
}
