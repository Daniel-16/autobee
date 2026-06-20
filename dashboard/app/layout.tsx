import "./globals.css";
import type { Metadata } from "next";

export const metadata: Metadata = {
  title: "AUTOBEE · Mission Control",
  description:
    "AutoBee autonomous surveillance drone — mission control. Natural-language intent, autonomous execution.",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body className="min-h-screen bg-bg font-sans text-accent antialiased selection:bg-white/20">
        {/* Deep-space backdrop layers (fixed, behind everything). */}
        <div className="space-bg" />
        <div className="space-grid" />
        {children}
      </body>
    </html>
  );
}
