#!/usr/bin/env python3
"""Convert the GNSSPeriph architecture markdown doc to a styled HTML page.

Uses the same visual style as the other reports on the user's desktop so
everything looks consistent in their browser.
"""
import os
import sys

SRC = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/Here4_GNSSPeriph_architecture.md"
DST = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/Here4_GNSSPeriph_architecture.html"

try:
    import markdown
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", "markdown"])
    import markdown

with open(SRC, "r", encoding="utf-8") as f:
    md_text = f.read()

# Render with table, fenced code, and TOC extensions
html_body = markdown.markdown(
    md_text,
    extensions=["tables", "fenced_code", "toc"],
)

css = """
body { font-family: -apple-system, "Segoe UI", Arial, sans-serif; max-width: 1100px;
       margin: 24px auto; padding: 0 18px; color: #222; line-height: 1.55; }
h1 { border-bottom: 3px solid #1f4e8f; padding-bottom: 4px; color: #1f4e8f; }
h2 { color: #1f4e8f; margin-top: 36px; border-bottom: 1px solid #ccc; padding-bottom: 2px; }
h3 { color: #444; margin-top: 28px; }
h4 { color: #666; margin-top: 20px; }
table { border-collapse: collapse; margin: 12px 0; }
th, td { border: 1px solid #c0c0c0; padding: 6px 10px; text-align: left;
         vertical-align: top; font-size: 13.5px; }
th { background: #eaeef5; }
code { background: #f4f4f4; padding: 1px 4px; border-radius: 3px;
       font-size: 12.5px; font-family: Consolas, monospace; }
pre { background: #f6f8fa; border: 1px solid #d0d7de; padding: 12px;
      border-radius: 6px; overflow-x: auto; font-size: 12.5px;
      font-family: Consolas, monospace; line-height: 1.4; }
pre code { background: transparent; padding: 0; font-size: inherit; }
ul, ol { padding-left: 22px; }
li { margin: 3px 0; }
hr { border: none; border-top: 1px solid #ddd; margin: 24px 0; }
a { color: #1f4e8f; }
.toc { background: #f4f7fc; border: 1px solid #c8d4e8; padding: 12px 22px;
       border-radius: 4px; margin: 18px 0; }
.toc ul { margin: 4px 0; }
strong { color: #2a2a2a; }
"""

html_full = f"""<!doctype html>
<html><head><meta charset="utf-8">
<title>Here4 GNSSPeriph Firmware Architecture</title>
<style>{css}</style>
</head><body>
{html_body}
</body></html>
"""

with open(DST, "w", encoding="utf-8") as f:
    f.write(html_full)

print(f"Wrote {DST}  ({os.path.getsize(DST):,} bytes)")
