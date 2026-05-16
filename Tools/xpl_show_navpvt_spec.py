"""Find and print the NAV-PVT spec page from the u-blox PDF."""
import pypdf

PDF = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/u-blox-F9-HPG-L1L5-1.40_InterfaceDescription_UBX-23006991.pdf"

r = pypdf.PdfReader(PDF)
# search for the actual table page
for i, p in enumerate(r.pages):
    t = p.extract_text() or ""
    has_pvt = "NAV-PVT" in t or "Navigation position velocity time" in t
    has_fields = "iTOW" in t and "numSV" in t
    if has_pvt and has_fields:
        print(f"Found NAV-PVT spec table on PDF page {i+1}")
        # Truncate and clean
        cleaned = t.encode("ascii", "replace").decode("ascii")
        # Trim leading metadata noise
        # Show first 5000 chars to get the full table
        print(cleaned[:5000])
        print("\n=========================== (page break here) ===========================\n")
        # Continue to next page in case table spans
        if i + 1 < len(r.pages):
            t2 = r.pages[i + 1].extract_text() or ""
            cleaned2 = t2.encode("ascii", "replace").decode("ascii")
            print(cleaned2[:3000])
        break
else:
    print("NAV-PVT spec page not found via primary search.")
    # Try alternate: search for "0x01 0x07" and gSpeed
    for i, p in enumerate(r.pages):
        t = p.extract_text() or ""
        if "gSpeed" in t and ("0x07" in t or "PVT" in t):
            print(f"Possible candidate on page {i+1}: {t[:200]}")
