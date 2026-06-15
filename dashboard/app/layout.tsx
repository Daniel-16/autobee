import "./globals.css";
import type { Metadata } from "next";

export const metadata: Metadata = {
  title: "AutoBee · Mission Control",
  description: "AutoBee autonomous surveillance drone — mission control dashboard",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body className="min-h-screen bg-bg font-sans text-slate-100 antialiased">
        {children}
      </body>
    </html>
  );
}
