import type { Config } from "tailwindcss";

const config: Config = {
  content: [
    "./app/**/*.{ts,tsx}",
    "./components/**/*.{ts,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        // SpaceX/Starlink monochrome stack — pure black canvas, near-black
        // panels, hairline separators. Status colour is the only chroma.
        bg: "#000000",
        ink: {
          950: "#050506",
          900: "#0a0a0c",
          850: "#0e0f12",
          800: "#141519",
          700: "#1b1d22",
        },
        panel: "#0a0a0c",
        panel2: "#101116",
        line: "rgba(255,255,255,0.08)",
        line2: "rgba(255,255,255,0.14)",
        accent: "#e8eaed",   // near-white (primary interactive)
        accent2: "#7c93ad",  // cool steel (links / quiet highlight)
        ok: "#3ddc97",
        warn: "#f5b94a",
        danger: "#ff5470",
      },
      fontFamily: {
        sans: ['"Saira"', "system-ui", "-apple-system", "Segoe UI",
               "Roboto", "Helvetica", "Arial", "sans-serif"],
        display: ['"Saira Condensed"', '"Saira"', "system-ui", "sans-serif"],
      },
      letterSpacing: {
        wider2: "0.18em",
        wider3: "0.28em",
      },
      keyframes: {
        "fade-up": {
          "0%": { opacity: "0", transform: "translateY(24px)" },
          "100%": { opacity: "1", transform: "translateY(0)" },
        },
        float: {
          "0%,100%": { transform: "translateY(-8px)" },
          "50%": { transform: "translateY(8px)" },
        },
        "pulse-ring": {
          "0%": { transform: "scale(0.9)", opacity: "0.7" },
          "70%,100%": { transform: "scale(2.2)", opacity: "0" },
        },
      },
      animation: {
        "fade-up": "fade-up 0.8s cubic-bezier(0.16,1,0.3,1) both",
        float: "float 6s ease-in-out infinite",
        "pulse-ring": "pulse-ring 2.4s cubic-bezier(0.4,0,0.6,1) infinite",
      },
    },
  },
  plugins: [],
};

export default config;
