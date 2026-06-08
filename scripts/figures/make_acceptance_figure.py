#!/usr/bin/env python3
"""Render the acceptance harness results into a thesis-ready figure + tables.

Reads the JSONL written by scripts/acceptance/run_acceptance.py and emits:
    docs/figures/acceptance_summary.svg   status chart (one row per scenario + TSR)
    docs/figures/acceptance_table.tex     LaTeX booktabs table
    docs/figures/acceptance_table.md       Markdown table

It only visualises whatever is in the JSONL — it never invents a result.
After running it, convert the SVG on the host:
    rsvg-convert -f pdf -o docs/figures/acceptance_summary.pdf docs/figures/acceptance_summary.svg
    rsvg-convert -f png -z 2 -o docs/figures/acceptance_summary.png docs/figures/acceptance_summary.svg

Usage:
    python3 scripts/figures/make_acceptance_figure.py
    python3 scripts/figures/make_acceptance_figure.py --in acceptance_results.jsonl --outdir docs/figures
"""

import argparse
import json
import os

# Short scenario descriptions, kept in sync with run_acceptance.py SCENARIOS.
DESCRIPTIONS = {
    "T-01": "Scan convocation ground for overcrowding",
    "T-02": "Check library entrance for gatherings",
    "T-03": "Return to base immediately",
    "T-04": "Patrol a named landmark",
    "T-05": "Gibberish input must not crash",
    "T-06": "Perception kill triggers emergency land",
    "T-07": "High density fires anomaly alert",
    "T-08": "SLAM ATE < 0.5 m on 50 m corridor",
    "T-09": "Local-area survey (demo-day acceptance)",
}

STYLE = {
    "PASS": ("#e8f5e9", "#2e7d32", "#1f5d23"),
    "FAIL": ("#fdecec", "#c0392b", "#922b21"),
    "SKIP": ("#eef0f2", "#888888", "#555555"),
}


def load_records(path):
    records = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def tally(records):
    p = sum(1 for r in records if r.get("result") == "PASS")
    f = sum(1 for r in records if r.get("result") == "FAIL")
    s = sum(1 for r in records if r.get("result") == "SKIP")
    tsr = p / max(1, p + f) * 100.0
    return p, f, s, tsr


def _xml(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def first_note(rec):
    fails = rec.get("failures") or []
    if rec.get("result") == "PASS":
        return ""
    if rec.get("result") == "SKIP":
        return fails[0] if fails else "skipped"
    return fails[0] if fails else "fail (no reason recorded)"


def build_svg(records):
    p, f, s, tsr = tally(records)
    row_h = 44
    top = 132
    width = 860
    height = top + len(records) * row_h + 56
    banner_ok = tsr >= 80.0 and f == 0
    banner_fill = "#e8f5e9" if banner_ok else "#fff4e5"
    banner_stroke = "#2e7d32" if banner_ok else "#c98a1b"
    banner_text = "#1f5d23" if banner_ok else "#8a5a12"

    out = []
    out.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" '
        f'font-family="\'DejaVu Sans\', Helvetica, Arial, sans-serif">'
    )
    out.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>')
    out.append('<text x="40" y="40" font-size="18" font-weight="bold" fill="#1a1a1a">'
               'Acceptance test results (PRD &#167;11, T-01–T-09)</text>')

    # TSR banner
    out.append(f'<rect x="40" y="60" width="{width-80}" height="48" rx="8" '
               f'fill="{banner_fill}" stroke="{banner_stroke}" stroke-width="1.6"/>')
    out.append(f'<text x="60" y="90" font-size="17" font-weight="bold" fill="{banner_text}">'
               f'TSR = {tsr:.0f}%</text>')
    out.append(f'<text x="170" y="90" font-size="13.5" fill="{banner_text}">'
               f'pass {p} &#183; fail {f} &#183; skip {s} &#183; target ≥ 80%'
               f'{"" if (p+f) else " &#183; (no scored scenarios)"}</text>')

    # column headers
    out.append('<text x="44" y="126" font-size="11.5" font-weight="bold" fill="#555">ID</text>')
    out.append('<text x="112" y="126" font-size="11.5" font-weight="bold" fill="#555">RESULT</text>')
    out.append('<text x="210" y="126" font-size="11.5" font-weight="bold" fill="#555">SCENARIO</text>')
    out.append(f'<line x1="40" y1="{top-10}" x2="{width-40}" y2="{top-10}" '
               f'stroke="#cccccc" stroke-width="1.2"/>')

    for i, rec in enumerate(records):
        y = top + i * row_h
        rid = rec.get("id", "?")
        result = rec.get("result", "?")
        fill, stroke, txt = STYLE.get(result, ("#f0f0f0", "#999", "#444"))
        desc = DESCRIPTIONS.get(rid, "")
        note = first_note(rec)
        # row separator (between rows only, not above the first)
        if i > 0:
            out.append(f'<line x1="40" y1="{y-8}" x2="{width-40}" y2="{y-8}" '
                       f'stroke="#eeeeee" stroke-width="1"/>')
        out.append(f'<text x="44" y="{y+14}" font-size="13" font-weight="bold" fill="#222">{_xml(rid)}</text>')
        # status pill
        out.append(f'<rect x="104" y="{y}" width="64" height="22" rx="11" '
                   f'fill="{fill}" stroke="{stroke}" stroke-width="1.4"/>')
        out.append(f'<text x="136" y="{y+15}" font-size="12" font-weight="bold" '
                   f'fill="{txt}" text-anchor="middle">{_xml(result)}</text>')
        # description
        out.append(f'<text x="210" y="{y+13}" font-size="12.5" fill="#222">{_xml(desc)}</text>')
        if note:
            out.append(f'<text x="210" y="{y+30}" font-size="10.5" fill="{stroke}">{_xml(note)}</text>')

    out.append('</svg>')
    return "\n".join(out)


def build_markdown(records):
    p, f, s, tsr = tally(records)
    lines = ["| ID | Scenario | Result | Notes |", "|---|---|---|---|"]
    for rec in records:
        rid = rec.get("id", "?")
        lines.append(
            f"| {rid} | {DESCRIPTIONS.get(rid, '')} | {rec.get('result', '?')} "
            f"| {first_note(rec) or '—'} |"
        )
    lines.append("")
    lines.append(f"**TSR = {tsr:.0f}%** (pass {p}, fail {f}, skip {s}; target ≥ 80%).")
    return "\n".join(lines) + "\n"


def _tex_escape(t):
    return (str(t).replace("\\", r"\textbackslash{}").replace("_", r"\_")
            .replace("&", r"\&").replace("%", r"\%").replace("#", r"\#"))


def build_latex(records):
    p, f, s, tsr = tally(records)
    rows = []
    for rec in records:
        rid = rec.get("id", "?")
        note = first_note(rec) or "--"
        rows.append(f"{_tex_escape(rid)} & {_tex_escape(DESCRIPTIONS.get(rid, ''))} "
                    f"& {_tex_escape(rec.get('result', '?'))} & {_tex_escape(note)} \\\\")
    body = "\n".join(rows)
    return (
        "% requires \\usepackage{booktabs}\n"
        "\\begin{table}[t]\n  \\centering\n"
        "  \\caption{Acceptance test outcomes (PRD~\\S11). "
        f"TSR $= {tsr:.0f}\\%$ (pass {p}, fail {f}, skip {s}; target $\\geq 80\\%$).}}\n"
        "  \\label{tab:acceptance}\n"
        "  \\begin{tabular}{llll}\n    \\toprule\n"
        "    ID & Scenario & Result & Notes \\\\\n    \\midrule\n"
        f"    {body}\n"
        "    \\bottomrule\n  \\end{tabular}\n\\end{table}\n"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="infile", default="acceptance_results.jsonl")
    ap.add_argument("--outdir", default="docs/figures")
    args = ap.parse_args()

    if not os.path.isfile(args.infile):
        raise SystemExit(f"no results file at {args.infile!r}; run scripts/acceptance/run_acceptance.py first")

    records = load_records(args.infile)
    if not records:
        raise SystemExit(f"{args.infile!r} is empty")

    os.makedirs(args.outdir, exist_ok=True)
    svg_path = os.path.join(args.outdir, "acceptance_summary.svg")
    md_path = os.path.join(args.outdir, "acceptance_table.md")
    tex_path = os.path.join(args.outdir, "acceptance_table.tex")

    with open(svg_path, "w") as fh:
        fh.write(build_svg(records))
    with open(md_path, "w") as fh:
        fh.write(build_markdown(records))
    with open(tex_path, "w") as fh:
        fh.write(build_latex(records))

    p, f, s, tsr = tally(records)
    print(f"read {len(records)} scenario record(s) from {args.infile}")
    print(f"TSR = {tsr:.0f}%  (pass {p}, fail {f}, skip {s})")
    print(f"wrote {svg_path}\n      {md_path}\n      {tex_path}")
    print("\nNext (on host): rsvg-convert -f pdf -o "
          f"{os.path.join(args.outdir, 'acceptance_summary.pdf')} {svg_path}")


if __name__ == "__main__":
    main()
