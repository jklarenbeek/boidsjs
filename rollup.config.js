import resolve from 'rollup-plugin-node-resolve';

export default {
  input: 'src/index.js',
  output: {
    file: 'public/index.js',
    format: 'esm'
  },
  plugins: [ resolve() ],
}
