"""Dump the spec PDF pages that define NAV-PVT and MON-HW so user can verify."""
import pypdf

PDF = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/u-blox-F9-HPG-L1L5-1.40_InterfaceDescription_UBX-23006991.pdf"

r = pypdf.PdfReader(PDF)
print(f"PDF: {PDF}")
print(f"Total pages: {len(r.pages)}\n")

# Find pages with the field tables
nav_pvt_pages = []
mon_hw_pages = []
checksum_pages = []
for i, p in enumerate(r.pages):
    t = p.extract_text() or ""
    if ("UBX-NAV-PVT" in t or "NAV-PVT" in t) and "gSpeed" in t:
        nav_pvt_pages.append((i + 1, t))
    elif ("UBX-NAV-PVT" in t or "NAV-PVT" in t) and "fixType" in t and "numSV" in t:
        nav_pvt_pages.append((i + 1, t))
    if "UBX-MON-HW" in t and "agcCnt" in t and "noisePerMS" in t:
        mon_hw_pages.append((i + 1, t))
    if "UBX checksum" in t and "Fletcher" in t:
        checksum_pages.append((i + 1, t))

print(f"NAV-PVT field-table page candidates: {[p[0] for p in nav_pvt_pages]}")
print(f"MON-HW field-table page candidates:  {[p[0] for p in mon_hw_pages]}")
print(f"Checksum spec page candidates:       {[p[0] for p in checksum_pages]}")

if nav_pvt_pages:
    pgnum, t = nav_pvt_pages[0]
    print("\n" + "=" * 70)
    print(f"NAV-PVT FIELD TABLE - PDF PAGE {pgnum}")
    print("=" * 70)
    print(t[:3500].encode("ascii", "replace").decode("ascii"))

if mon_hw_pages:
    pgnum, t = mon_hw_pages[0]
    print("\n" + "=" * 70)
    print(f"MON-HW FIELD TABLE - PDF PAGE {pgnum}")
    print("=" * 70)
    print(t[:3500].encode("ascii", "replace").decode("ascii"))

if checksum_pages:
    pgnum, t = checksum_pages[0]
    print("\n" + "=" * 70)
    print(f"UBX CHECKSUM SPEC - PDF PAGE {pgnum}")
    print("=" * 70)
    print(t[:2500].encode("ascii", "replace").decode("ascii"))
