/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        bgMain: '#F1F5F9',       // App background
        panelWhite: '#FFFFFF',   // Panel background
        textMain: '#1F2937',     // Primary text
        textMuted: '#6B7280',    // Secondary/Unit text
        aeroCyan: '#0CA5E9',     // Brand/Accent
        statusGreen: '#10B981',  // Connected/Safe
        alertRed: '#EF4444',     // Warning
        borderGray: '#E5E7EB',   // Panel borders
      },
      fontFamily: {
        sans: ['Inter', 'sans-serif'],
      },
      fontSize: {
        'xxs': '0.65rem',
      }
    },
  },
  plugins: [],
}
