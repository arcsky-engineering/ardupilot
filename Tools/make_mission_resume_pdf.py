"""Generate a field-ready PDF guide for the Xplorer mission resume feature."""
from pathlib import Path
from reportlab.lib.pagesizes import LETTER
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether,
)
from reportlab.lib.enums import TA_LEFT


DESKTOP = Path.home() / "Desktop"
OUT = DESKTOP / "Mission_Resume_Field_Guide.pdf"


styles = getSampleStyleSheet()
H1 = ParagraphStyle("H1", parent=styles["Heading1"], fontSize=16, spaceAfter=6, textColor=colors.HexColor("#1a3a6c"))
H2 = ParagraphStyle("H2", parent=styles["Heading2"], fontSize=12, spaceBefore=10, spaceAfter=4, textColor=colors.HexColor("#1a3a6c"))
BODY = ParagraphStyle("Body", parent=styles["BodyText"], fontSize=9.5, leading=12, alignment=TA_LEFT)
SMALL = ParagraphStyle("Small", parent=BODY, fontSize=8.5, leading=10.5, textColor=colors.HexColor("#444444"))
MONO = ParagraphStyle("Mono", parent=BODY, fontName="Courier", fontSize=8.5, leading=11)
CHECK = ParagraphStyle("Check", parent=BODY, fontSize=9.5, leading=14, leftIndent=18, firstLineIndent=-18)


def p(text, style=BODY):
    return Paragraph(text, style)


def kv_table(rows, col_widths):
    t = Table(rows, colWidths=col_widths, hAlign="LEFT")
    t.setStyle(TableStyle([
        ("FONTSIZE", (0, 0), (-1, -1), 9),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1a3a6c")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.HexColor("#f3f6fb"), colors.white]),
        ("INNERGRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#cccccc")),
        ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#1a3a6c")),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    return t


def check_item(text):
    return p(f"&#9744;&nbsp;&nbsp;{text}", CHECK)


def hr():
    t = Table([[""]], colWidths=[6.5 * inch], rowHeights=[0.5])
    t.setStyle(TableStyle([("LINEABOVE", (0, 0), (-1, 0), 0.5, colors.HexColor("#cccccc"))]))
    return t


story = []

# Title
story.append(p("Mission Resume — Field Guide", H1))
story.append(p("Xplorer ArduCopter custom build &nbsp;&middot;&nbsp; Branch: <b>xplorer/4.6.2-custom</b>", SMALL))
story.append(Spacer(1, 8))

# Parameters
story.append(p("Parameters", H2))
param_rows = [
    ["Param", "R/W", "Purpose"],
    [p("<b>MIS_RESTART</b>", MONO), "RW",
     p("<b>0</b> = Resume from saved index (default).<br/><b>1</b> = Restart from beginning.", BODY)],
    [p("<b>MIS_LAST_INDEX</b>", MONO), "RO",
     p("NVM-saved index of the <b>previous</b> waypoint. Used so the full leg is re-flown on resume "
       "(prevents missing data on a survey line). Auto-managed.", BODY)],
    [p("<b>MIS_OPTIONS</b> bit 0", MONO), "RW",
     p("If set, mission AND <font face='Courier'>MIS_LAST_INDEX</font> are wiped on every boot.", BODY)],
]
story.append(kv_table(param_rows, [1.5 * inch, 0.5 * inch, 4.5 * inch]))

# Behavior at a glance
story.append(p("Behavior at a Glance", H2))

save_rows = [
    ["When", "Action"],
    [p("Exit AUTO mode", BODY),
     p("<font face='Courier'>stop()</font> saves <font face='Courier'>_prev_nav_cmd_wp_index</font> "
       "to <font face='Courier'>MIS_LAST_INDEX</font>.", BODY)],
    [p("Disarm while in AUTO", BODY),
     p("<font face='Courier'>check_and_save_on_disarm()</font> saves "
       "<font face='Courier'>_prev_nav_cmd_wp_index</font>.", BODY)],
    [p("Mission complete / landing / RTL path", BODY),
     p("<font face='Courier'>MIS_LAST_INDEX</font> cleared to 0.", BODY)],
    [p("MIS_RESTART=1 enters AUTO", BODY),
     p("<font face='Courier'>start()</font> clears <font face='Courier'>MIS_LAST_INDEX</font>, "
       "begins at WP 1.", BODY)],
    [p("Mission upload / CLEAR_ON_BOOT", BODY),
     p("<font face='Courier'>clear()</font> clears <font face='Courier'>MIS_LAST_INDEX</font>.", BODY)],
    [p("Cold-boot resume (MIS_RESTART=0)", BODY),
     p("<font face='Courier'>resume()</font> reads <font face='Courier'>MIS_LAST_INDEX</font> "
       "and calls <font face='Courier'>set_current_cmd()</font> on it.", BODY)],
]
story.append(kv_table(save_rows, [2.0 * inch, 4.5 * inch]))

# GCS messages
story.append(p("GCS Status Messages to Watch For", H2))
msg_rows = [
    ["Message", "Means"],
    [p("Mission: Saved index N for resume", MONO),
     p("NVM was just written with previous WP index.", BODY)],
    [p("Mission: Resuming from saved index N", MONO),
     p("Cold-boot resume — vehicle is heading to WP N.", BODY)],
    [p("Mission: Cleared saved index ...", MONO),
     p("NVM was zeroed (mission complete, landing, or no active WP).", BODY)],
]
story.append(kv_table(msg_rows, [3.0 * inch, 3.5 * inch]))

# Caveats
story.append(p("Caveats", H2))
story.append(p(
    "<b>Auto-RTL forces resume.</b> Entering Auto-RTL sets <font face='Courier'>_force_resume=true</font>, "
    "which makes the next <font face='Courier'>start_or_resume()</font> call <font face='Courier'>resume()</font> "
    "even if <font face='Courier'>MIS_RESTART=1</font>. The flag is single-use.",
    BODY))
story.append(Spacer(1, 4))
story.append(p(
    "<b>Disarm save only fires in AUTO.</b> <font face='Courier'>check_and_save_on_disarm()</font> "
    "is called from <font face='Courier'>mission.update()</font>, which only runs in AUTO. "
    "Disarms outside AUTO are covered by the earlier <font face='Courier'>stop()</font> on AUTO exit.",
    BODY))

story.append(PageBreak())

# Test plan
story.append(p("Field Test Plan", H1))
story.append(p("Run these in order. Have the GCS messages window open. Use a 5-6 WP mission.", SMALL))
story.append(Spacer(1, 6))

# Test 1
story.append(p("Test 1 &mdash; Save fires on AUTO exit", H2))
story.append(check_item("Set <font face='Courier'>MIS_RESTART=0</font>, write, reboot."))
story.append(check_item("Arm, switch to AUTO, fly past WP 3."))
story.append(check_item("Switch to LOITER. Watch for <font face='Courier'>Mission: Saved index 2 for resume</font>."))
story.append(check_item("Read <font face='Courier'>MIS_LAST_INDEX</font> &rarr; expect <b>2</b>."))
story.append(check_item("Result: <b>PASS / FAIL</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Notes: ____________________"))

story.append(hr())

# Test 2
story.append(p("Test 2 &mdash; Resume across power cycle", H2))
story.append(check_item("Continuing from Test 1 (don&rsquo;t clear NVM): power cycle the FC."))
story.append(check_item("Read <font face='Courier'>MIS_LAST_INDEX</font> &rarr; still <b>2</b>."))
story.append(check_item("Arm, switch to AUTO. Watch for <font face='Courier'>Mission: Resuming from saved index 2</font>."))
story.append(check_item("Vehicle flies <b>back to WP 2</b> first (re-flying the leg), then continues forward."))
story.append(check_item("Result: <b>PASS / FAIL</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Notes: ____________________"))

story.append(hr())

# Test 3
story.append(p("Test 3 &mdash; MIS_RESTART=1 actually restarts (the original bug)", H2))
story.append(check_item("With <font face='Courier'>MIS_LAST_INDEX</font> non-zero from Test 2, set <font face='Courier'>MIS_RESTART=1</font>, write."))
story.append(check_item("Arm, AUTO. Vehicle goes to <b>WP 1</b>, NOT to <font face='Courier'>MIS_LAST_INDEX</font>."))
story.append(check_item("Read <font face='Courier'>MIS_LAST_INDEX</font> &rarr; expect <b>0</b> (cleared by <font face='Courier'>start()</font>)."))
story.append(check_item("Result: <b>PASS / FAIL</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Notes: ____________________"))

story.append(hr())

# Test 4
story.append(p("Test 4 &mdash; Completion clears the index", H2))
story.append(check_item("Set <font face='Courier'>MIS_RESTART=1</font>, fly the entire mission to completion (or landing)."))
story.append(check_item("Disarm. Watch for <font face='Courier'>Mission: Cleared saved index (mission complete or landing)</font>."))
story.append(check_item("Read <font face='Courier'>MIS_LAST_INDEX</font> &rarr; expect <b>0</b>."))
story.append(check_item("Result: <b>PASS / FAIL</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Notes: ____________________"))

story.append(hr())

# Test 5
story.append(p("Test 5 &mdash; New mission upload clears the index", H2))
story.append(check_item("Force <font face='Courier'>MIS_LAST_INDEX</font> non-zero (fly partway, switch out of AUTO)."))
story.append(check_item("Confirm <font face='Courier'>MIS_LAST_INDEX</font> &gt; 0."))
story.append(check_item("Upload a fresh mission from the GCS."))
story.append(check_item("Read <font face='Courier'>MIS_LAST_INDEX</font> &rarr; expect <b>0</b> (cleared by <font face='Courier'>clear()</font>)."))
story.append(check_item("Result: <b>PASS / FAIL</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Notes: ____________________"))

story.append(hr())

# Test 6
story.append(p("Test 6 (optional) &mdash; Re-fly the leg confirmation", H2))
story.append(check_item("Repeat Test 1 setup. After resume, verify the path on the GCS map."))
story.append(check_item("Vehicle should head <b>back to the previous WP</b> first, then continue (don&rsquo;t-miss-data behavior)."))
story.append(check_item("Result: <b>PASS / FAIL</b> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Notes: ____________________"))

story.append(Spacer(1, 12))
story.append(hr())
story.append(Spacer(1, 4))
story.append(p("Pilot: ___________________  &nbsp; Date: __________  &nbsp; Aircraft: __________  &nbsp; FW build: __________", SMALL))


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(colors.HexColor("#888888"))
    canvas.drawString(0.75 * inch, 0.4 * inch, "Mission Resume Field Guide  -  Xplorer ArduCopter")
    canvas.drawRightString(LETTER[0] - 0.75 * inch, 0.4 * inch, f"Page {doc.page}")
    canvas.restoreState()


doc = SimpleDocTemplate(
    str(OUT),
    pagesize=LETTER,
    leftMargin=0.75 * inch,
    rightMargin=0.75 * inch,
    topMargin=0.6 * inch,
    bottomMargin=0.6 * inch,
    title="Mission Resume Field Guide",
    author="Xplorer",
)
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print(f"Wrote: {OUT}")
