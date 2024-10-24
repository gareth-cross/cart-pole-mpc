module.exports = {
  content: ['./src/*.html'],
  theme: {
    extend: {},
    screens: {
      sm: '700px',
      md: '768px',
      lg: '1024px',
      xl: '1280px',
      '2xl': '1536px'
    }
  },
  variants: {
    extend: {}
  },
  plugins: [],
  safelist: ['text-slate-100', 'w-full', 'h-full', 'pr-2', 'pt-1']
};
