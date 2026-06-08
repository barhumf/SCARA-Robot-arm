#!/usr/bin/env python3
"""
SCARA Robotic Arm — Drawing UI
Hardware: 7-inch 2-link arm (88.9 + 88.9 mm), NEMA-17 + TMC2209 + ESP32
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math, os


# ── Palette & style ───────────────────────────────────────────────────────────
BG          = "#0f1117"
BG2         = "#181c25"
BG3         = "#1f2433"
BORDER      = "#2a3045"
ACCENT      = "#00d4ff"
ACCENT2     = "#ff6b35"
TEXT        = "#e8eaf0"
TEXT2       = "#7a8099"
STROKE_CLR  = "#00d4ff"
PREVIEW_CLR = "#ff6b35"

FONT_TITLE  = ("Courier New", 11, "bold")
FONT_LABEL  = ("Courier New", 9)
FONT_SMALL  = ("Courier New", 8)
FONT_BTN    = ("Courier New", 9, "bold")
FONT_MONO   = ("Courier New", 9)

# ── Config ────────────────────────────────────────────────────────────────────
CANVAS_W      = 476
CANVAS_H      = 616
WORK_W_MM     = 215.9
WORK_H_MM     = 279.4
WORK_OFFSET_X = 88.9
WORK_OFFSET_Y = 0.0
FEED_DRAW     = 600
CURVE_STEPS   = 40

# ── Coord helpers ─────────────────────────────────────────────────────────────
def canvas_to_mm(px, py):
    mx = (px / CANVAS_W) * WORK_W_MM + WORK_OFFSET_X - WORK_W_MM / 2.0
    my = ((CANVAS_H - py) / CANVAS_H) * WORK_H_MM + WORK_OFFSET_Y - WORK_H_MM / 2.0
    return mx, my

# ── Bezier ────────────────────────────────────────────────────────────────────
def bezier_points(p0, p1, p2, steps=CURVE_STEPS):
    return [
        ((1-t)**2*p0[0] + 2*(1-t)*t*p1[0] + t**2*p2[0],
         (1-t)**2*p0[1] + 2*(1-t)*t*p1[1] + t**2*p2[1])
        for t in (i/steps for i in range(steps+1))
    ]

# ── Shape generators ──────────────────────────────────────────────────────────
def make_circle(cx, cy, r, steps=24):   # 24 pts — faster than 72, still smooth
    return [(cx + r*math.cos(2*math.pi*i/steps),
             cy + r*math.sin(2*math.pi*i/steps)) for i in range(steps+1)]

def _interp(p1, p2, n=4):
    """Insert n-1 intermediate points between two canvas points for straight lines."""
    return [(p1[0] + (p2[0]-p1[0])*i/n,
             p1[1] + (p2[1]-p1[1])*i/n) for i in range(1, n+1)]

def make_square(cx, cy, h):
    corners = [(cx-h,cy-h),(cx+h,cy-h),(cx+h,cy+h),(cx-h,cy+h),(cx-h,cy-h)]
    pts = [corners[0]]
    for i in range(1, len(corners)):
        pts.extend(_interp(corners[i-1], corners[i]))
    return pts

def make_triangle(cx, cy, r):
    corners = [(cx + r*math.cos(math.pi/2 + 2*math.pi*i/3),
                cy - r*math.sin(math.pi/2 + 2*math.pi*i/3)) for i in range(4)]
    pts = [corners[0]]
    for i in range(1, len(corners)):
        pts.extend(_interp(corners[i-1], corners[i]))
    return pts

def make_star(cx, cy, r, ri=None, n=5):
    ri = ri or r*0.4
    raw = []
    for i in range(n*2+1):
        a = math.pi/2 + math.pi*i/n
        rad = r if i%2==0 else ri
        raw.append((cx + rad*math.cos(a), cy - rad*math.sin(a)))
    pts = [raw[0]]
    for i in range(1, len(raw)):
        pts.extend(_interp(raw[i-1], raw[i]))
    return pts

# ── G-code ────────────────────────────────────────────────────────────────────
def generate_gcode(strokes, label="Drawing", feed_draw=FEED_DRAW):
    out = [
        f"; SCARA G-code — {label}",
        f"; Links 88.9+88.9 mm | Work {WORK_W_MM:.1f}×{WORK_H_MM:.1f} mm",
        "", "G28 ; Home", "M5  ; Pen up", "",
    ]
    for i, s in enumerate(strokes):
        pts = s["points"]
        if len(pts) < 2: continue
        out.append(f"; [{s['type'].upper()} {i+1}]")
        x0, y0 = canvas_to_mm(*pts[0])
        out += ["M5", f"G0 X{x0:.2f} Y{y0:.2f}", "M3"]
        for px, py in pts[1:]:
            x, y = canvas_to_mm(px, py)
            out.append(f"G1 X{x:.2f} Y{y:.2f} F{feed_draw}")
        out.append("")
    out += ["M5 ; Pen up", "G28 ; Home", "M0  ; Motors off", ""]
    return "\n".join(out)

def _send_via_serial(gcode, port, status_cb):
    try:
        import serial, time
    except ImportError:
        messagebox.showerror("Missing", "pip install pyserial"); return
    try:
        ser = serial.Serial(port, 115200, timeout=10)
    except Exception as e:
        messagebox.showerror("Serial error", str(e)); return
    time.sleep(2)
    while ser.in_waiting: ser.readline()
    lines = [l for l in gcode.splitlines() if l.strip() and not l.strip().startswith(';')]
    for idx, line in enumerate(lines):
        ser.write((line+'\n').encode())
        status_cb(f"[{idx+1}/{len(lines)}] {line}")
        deadline = time.time() + 30
        while time.time() < deadline:
            if ser.readline().decode(errors='replace').strip().lower().startswith('ok'): break
        else:
            status_cb(f"timeout: {line}")
    ser.close()
    status_cb("Complete.")

# ── Tool constants ────────────────────────────────────────────────────────────
TOOL_LINE     = "line"
TOOL_CURVE    = "curve"
TOOL_CIRCLE   = "circle"
TOOL_SQUARE   = "square"
TOOL_TRIANGLE = "triangle"
TOOL_STAR     = "star"
SHAPE_TOOLS   = {TOOL_CIRCLE, TOOL_SQUARE, TOOL_TRIANGLE, TOOL_STAR}

TOOL_ICONS = {
    TOOL_LINE:     "╱",
    TOOL_CURVE:    "∿",
    TOOL_CIRCLE:   "○",
    TOOL_SQUARE:   "□",
    TOOL_TRIANGLE: "△",
    TOOL_STAR:     "✦",
}
TOOL_HINTS = {
    TOOL_LINE:     "Click start → click end",
    TOOL_CURVE:    "Click start → click end → drag bend → click",
    TOOL_CIRCLE:   "Click centre → drag radius → release",
    TOOL_SQUARE:   "Click centre → drag size → release",
    TOOL_TRIANGLE: "Click centre → drag radius → release",
    TOOL_STAR:     "Click centre → drag radius → release",
}

# ── Custom widgets ────────────────────────────────────────────────────────────
class FlatButton(tk.Label):
    def __init__(self, parent, text, command, bg=BG3, fg=TEXT,
                 hover_bg=ACCENT, hover_fg=BG, width=None, **kw):
        cfg = dict(bg=bg, fg=fg, font=FONT_BTN, cursor="hand2",
                   padx=12, pady=6, relief="flat")
        if width: cfg["width"] = width
        cfg.update(kw)
        super().__init__(parent, text=text, **cfg)
        self._bg = bg; self._fg = fg
        self._hbg = hover_bg; self._hfg = hover_fg
        self._cmd = command
        self.bind("<Enter>",          lambda e: self.config(bg=self._hbg, fg=self._hfg))
        self.bind("<Leave>",          lambda e: self.config(bg=self._bg,  fg=self._fg))
        self.bind("<ButtonPress-1>",  lambda e: self.config(bg=ACCENT2, fg=BG))
        self.bind("<ButtonRelease-1>",lambda e: (self.config(bg=self._hbg, fg=self._hfg),
                                                  self._cmd()))

class ToolButton(tk.Frame):
    def __init__(self, parent, icon, label, value, var, command):
        super().__init__(parent, bg=BG2, cursor="hand2")
        self._var = var; self._val = value; self._cmd = command
        self._active = False
        self.icon_lbl = tk.Label(self, text=icon, bg=BG2, fg=TEXT2,
                                 font=("Courier New", 14))
        self.icon_lbl.pack(pady=(6,1))
        self.text_lbl = tk.Label(self, text=label, bg=BG2, fg=TEXT2,
                                 font=FONT_SMALL)
        self.text_lbl.pack(pady=(0,6))
        for w in (self, self.icon_lbl, self.text_lbl):
            w.bind("<ButtonRelease-1>", self._on_click)
            w.bind("<Enter>", self._on_enter)
            w.bind("<Leave>", self._on_leave)

    def _on_click(self, e):
        self._var.set(self._val)
        self._cmd()

    def _on_enter(self, e):
        if not self._active:
            for w in (self, self.icon_lbl, self.text_lbl):
                w.config(bg=BG3)

    def _on_leave(self, e):
        if not self._active:
            for w in (self, self.icon_lbl, self.text_lbl):
                w.config(bg=BG2)

    def set_active(self, active):
        self._active = active
        bg  = ACCENT if active else BG2
        for w in (self, self.icon_lbl, self.text_lbl):
            w.config(bg=bg)
        self.icon_lbl.config(fg=BG if active else TEXT2)
        self.text_lbl.config(fg=BG if active else TEXT2)


class SectionHeader(tk.Frame):
    def __init__(self, parent, text):
        super().__init__(parent, bg=BG2)
        tk.Label(self, text=text, bg=BG2, fg=ACCENT,
                 font=FONT_LABEL).pack(side=tk.LEFT)
        tk.Frame(self, bg=BORDER, height=1).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(8,0), pady=6)


# ── Main App ──────────────────────────────────────────────────────────────────
class DrawingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA DRAW")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self.strokes      = []
        self.tool         = TOOL_LINE
        self.pen_width    = 1
        self.line_start   = None
        self.preview_id   = None
        self.curve_step   = 0
        self.curve_start  = None
        self.curve_end    = None
        self.curve_ctrl   = None
        self.curve_ids    = []
        self.shape_origin = None
        self.shape_ids    = []
        self._tool_btns   = {}

        self._build_ui()
        self._draw_grid()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        root = self.root

        topbar = tk.Frame(root, bg=BG, pady=0)
        topbar.pack(fill=tk.X)

        title_frame = tk.Frame(topbar, bg=BG, padx=16, pady=10)
        title_frame.pack(side=tk.LEFT)
        tk.Label(title_frame, text="SCARA", bg=BG, fg=ACCENT,
                 font=("Courier New", 16, "bold")).pack(side=tk.LEFT)
        tk.Label(title_frame, text=" DRAW", bg=BG, fg=TEXT,
                 font=("Courier New", 16, "bold")).pack(side=tk.LEFT)
        tk.Label(title_frame, text=" v2", bg=BG, fg=TEXT2,
                 font=("Courier New", 10)).pack(side=tk.LEFT, pady=(4,0))

        tk.Frame(topbar, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=4)

        btn_frame = tk.Frame(topbar, bg=BG, padx=10)
        btn_frame.pack(side=tk.LEFT, fill=tk.Y)
        actions = [
            ("SAVE  G-CODE",  self.save_gcode,     BG3, ACCENT),
            ("COPY  FOR CPP", self.copy_for_cpp,   BG3, ACCENT2),
            ("CALIBRATE",     self.open_calibrate, BG3, ACCENT),
        ]
        for txt, cmd, bg, hbg in actions:
            FlatButton(btn_frame, txt, cmd, bg=bg, hover_bg=hbg,
                       hover_fg=BG).pack(side=tk.LEFT, padx=4, pady=8)

        tk.Frame(topbar, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=4)

        edit_frame = tk.Frame(topbar, bg=BG, padx=10)
        edit_frame.pack(side=tk.LEFT, fill=tk.Y)
        FlatButton(edit_frame, "UNDO", self.undo_stroke,
                   bg=BG3, hover_bg=BG3, hover_fg=ACCENT2).pack(
                       side=tk.LEFT, padx=4, pady=8)
        FlatButton(edit_frame, "CLEAR", self.clear_canvas,
                   bg=BG3, hover_bg="#3a1515", hover_fg="#ff6060").pack(
                       side=tk.LEFT, padx=4, pady=8)

        self.status_var  = tk.StringVar(value="")
        self.strokes_var = tk.StringVar(value="0 strokes")
        right = tk.Frame(topbar, bg=BG, padx=16)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        tk.Label(right, textvariable=self.strokes_var, bg=BG, fg=TEXT2,
                 font=FONT_SMALL).pack(anchor="e", pady=(8,2))
        tk.Label(right, textvariable=self.status_var, bg=BG, fg=ACCENT2,
                 font=FONT_SMALL).pack(anchor="e", pady=(0,8))

        tk.Frame(root, bg=BORDER, height=1).pack(fill=tk.X)

        body = tk.Frame(root, bg=BG)
        body.pack(fill=tk.BOTH)

        tool_panel = tk.Frame(body, bg=BG2, width=88, pady=12)
        tool_panel.pack(side=tk.LEFT, fill=tk.Y)
        tool_panel.pack_propagate(False)

        tk.Label(tool_panel, text="TOOLS", bg=BG2, fg=TEXT2,
                 font=("Courier New", 7, "bold")).pack(pady=(0,8))

        self.tool_var = tk.StringVar(value=TOOL_LINE)
        tool_order = [TOOL_LINE, TOOL_CURVE, TOOL_CIRCLE,
                      TOOL_SQUARE, TOOL_TRIANGLE, TOOL_STAR]

        for val in tool_order:
            btn = ToolButton(tool_panel, TOOL_ICONS[val], val.upper(),
                             val, self.tool_var, self._on_tool_change)
            btn.pack(fill=tk.X, padx=6, pady=3)
            self._tool_btns[val] = btn

        canvas_wrap = tk.Frame(body, bg=BG, padx=12, pady=12)
        canvas_wrap.pack(side=tk.LEFT)

        canvas_border = tk.Frame(canvas_wrap, bg=BORDER, padx=1, pady=1)
        canvas_border.pack()

        self.canvas = tk.Canvas(canvas_border, width=CANVAS_W, height=CANVAS_H,
                                bg="#f8f7f2", cursor="crosshair",
                                highlightthickness=0)
        self.canvas.pack()

        self.canvas.bind("<ButtonPress-1>",   self.on_press)
        self.canvas.bind("<B1-Motion>",       self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>",          self.on_motion)
        self.canvas.bind("<ButtonPress-3>",   self.on_cancel)

        ruler = tk.Frame(canvas_wrap, bg=BG)
        ruler.pack(fill=tk.X, pady=(3,0))
        tk.Label(ruler, text="0\"",    bg=BG, fg=TEXT2, font=FONT_SMALL).pack(side=tk.LEFT)
        tk.Label(ruler, text="8.5\"",  bg=BG, fg=TEXT2, font=FONT_SMALL).pack(side=tk.RIGHT)
        tk.Label(ruler, text="4.25\"", bg=BG, fg=TEXT2, font=FONT_SMALL).pack()

        info = tk.Frame(body, bg=BG2, width=210, padx=14, pady=14)
        info.pack(side=tk.LEFT, fill=tk.Y)
        info.pack_propagate(False)

        def field(parent, label, value, vfg=TEXT):
            row = tk.Frame(parent, bg=BG2)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=label, bg=BG2, fg=TEXT2,
                     font=FONT_SMALL, width=10, anchor="w").pack(side=tk.LEFT)
            tk.Label(row, text=value, bg=BG2, fg=vfg,
                     font=FONT_MONO, anchor="w").pack(side=tk.LEFT)

        SectionHeader(info, "ARM CONFIG").pack(fill=tk.X, pady=(0,6))
        field(info, "L1",     "88.9 mm")
        field(info, "L2",     "88.9 mm")
        field(info, "REACH",  "177.8 mm", ACCENT)
        field(info, "AREA",   "8.5 × 11 in")
        field(info, "DRIVER", "TMC2209")
        field(info, "MCU",    "ESP32")

        tk.Frame(info, bg=BORDER, height=1).pack(fill=tk.X, pady=10)

        SectionHeader(info, "TOOL HINT").pack(fill=tk.X, pady=(0,6))
        self.hint_var = tk.StringVar(value=TOOL_HINTS[TOOL_LINE])
        tk.Label(info, textvariable=self.hint_var, bg=BG2, fg=TEXT,
                 font=FONT_SMALL, wraplength=178,
                 justify=tk.LEFT, anchor="nw").pack(fill=tk.X)

        tk.Frame(info, bg=BORDER, height=1).pack(fill=tk.X, pady=10)

        SectionHeader(info, "G-CODE CONFIG").pack(fill=tk.X, pady=(0,6))
        tk.Label(info, text="FEED DRAW (mm/min)", bg=BG2, fg=TEXT2,
                 font=FONT_SMALL, anchor="w").pack(fill=tk.X)
        self.feed_var = tk.IntVar(value=FEED_DRAW)
        feed_row = tk.Frame(info, bg=BG2); feed_row.pack(fill=tk.X, pady=2)
        self.feed_disp = tk.Label(feed_row, text=str(FEED_DRAW), bg=BG2,
                                  fg=ACCENT, font=FONT_MONO, width=5)
        self.feed_disp.pack(side=tk.RIGHT)
        tk.Scale(feed_row, from_=100, to=2000, orient=tk.HORIZONTAL,
                 variable=self.feed_var, bg=BG2, fg=TEXT2, troughcolor=BG3,
                 highlightthickness=0, bd=0, showvalue=False,
                 command=lambda v: self.feed_disp.config(text=v)
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        tk.Frame(info, bg=BORDER, height=1).pack(fill=tk.X, pady=10)

        SectionHeader(info, "SHORTCUTS").pack(fill=tk.X, pady=(0,6))
        for key, action in [("Ctrl+S", "Save G-code"),
                             ("Ctrl+Z", "Undo"),
                             ("Esc",    "Cancel / reset")]:
            row = tk.Frame(info, bg=BG2); row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=key, bg=BG3, fg=ACCENT,
                     font=FONT_SMALL, padx=4, pady=1).pack(side=tk.LEFT)
            tk.Label(row, text=f"  {action}", bg=BG2, fg=TEXT2,
                     font=FONT_SMALL).pack(side=tk.LEFT)

        tk.Frame(root, bg=BORDER, height=1).pack(fill=tk.X)
        statusbar = tk.Frame(root, bg=BG, pady=4)
        statusbar.pack(fill=tk.X)
        self.coord_var = tk.StringVar(value="X — mm  Y — mm")
        tk.Label(statusbar, textvariable=self.coord_var,
                 bg=BG, fg=TEXT2, font=FONT_SMALL, padx=12).pack(side=tk.LEFT)
        tk.Label(statusbar, text="SCARA DRAW  |  88.9+88.9 mm  |  8.5×11\"",
                 bg=BG, fg=TEXT2, font=FONT_SMALL, padx=12).pack(side=tk.RIGHT)

        root.bind("<Control-z>", lambda e: self.undo_stroke())
        root.bind("<Control-s>", lambda e: self.save_gcode())
        root.bind("<Escape>",    lambda e: self.on_cancel(None))

        self._update_tool_buttons()

    # ── Grid ──────────────────────────────────────────────────────────────────
    def _draw_grid(self):
        PPI = CANVAS_W / 8.5
        for i in range(1, 9):
            x = int(i * PPI)
            self.canvas.create_line(x, 0, x, CANVAS_H,
                                    fill="#e0ddd4" if i != 4 else "#ccc9bd",
                                    width=1, dash=(2,4) if i != 4 else (),
                                    tags="grid")
        PPV = CANVAS_H / 11.0
        for j in range(1, 11):
            y = int(j * PPV)
            self.canvas.create_line(0, y, CANVAS_W, y,
                                    fill="#e0ddd4" if j != 5 else "#ccc9bd",
                                    width=1, dash=(2,4) if j != 5 else (),
                                    tags="grid")
        bx = CANVAS_W // 2
        r  = 5
        self.canvas.create_oval(bx-r, CANVAS_H//2-r, bx+r, CANVAS_H//2+r,
                                fill="#cc4400", outline="#ff6633", tags="grid")
        self.canvas.create_text(bx+10, CANVAS_H//2, text="ARM BASE",
                                fill="#cc4400", font=("Courier New", 7),
                                anchor="w", tags="grid")
        self.canvas.create_rectangle(1, 1, CANVAS_W-1, CANVAS_H-1,
                                     outline="#aaa49a", width=1, tags="grid")

    # ── Tool management ───────────────────────────────────────────────────────
    def _on_tool_change(self):
        self.tool = self.tool_var.get()
        self._cancel_preview()
        self._update_tool_buttons()
        self.hint_var.set(TOOL_HINTS.get(self.tool, ""))
        self.status_var.set(TOOL_HINTS.get(self.tool, ""))

    def _update_tool_buttons(self):
        for val, btn in self._tool_btns.items():
            btn.set_active(val == self.tool)

    def _cancel_preview(self):
        if self.preview_id: self.canvas.delete(self.preview_id)
        self.preview_id   = None
        self.line_start   = None
        self.shape_origin = None
        for cid in self.curve_ids + self.shape_ids:
            self.canvas.delete(cid)
        self.curve_ids = []; self.shape_ids = []
        self.curve_step = 0
        self.curve_start = self.curve_end = self.curve_ctrl = None

    # ── Mouse events ──────────────────────────────────────────────────────────
    def on_press(self, event):
        if   self.tool in SHAPE_TOOLS: self.shape_origin = (event.x, event.y)
        elif self.tool == TOOL_LINE:   self._line_click(event.x, event.y)
        elif self.tool == TOOL_CURVE:  self._curve_click(event.x, event.y)

    def on_drag(self, event):
        self._update_coords(event.x, event.y)
        if self.tool in SHAPE_TOOLS and self.shape_origin:
            self._shape_preview(event.x, event.y)

    def on_release(self, event):
        if self.tool in SHAPE_TOOLS and self.shape_origin:
            self._shape_commit(event.x, event.y)

    def on_motion(self, event):
        self._update_coords(event.x, event.y)
        if   self.tool == TOOL_LINE:  self._line_motion(event.x, event.y)
        elif self.tool == TOOL_CURVE: self._curve_motion(event.x, event.y)

    def on_cancel(self, event):
        self._cancel_preview()
        self._on_tool_change()

    def _update_coords(self, px, py):
        mx, my = canvas_to_mm(px, py)
        self.coord_var.set(f"X {mx:+.1f} mm   Y {my:+.1f} mm")

    # ── Shape tools ───────────────────────────────────────────────────────────
    def _shape_r(self, x, y):
        cx, cy = self.shape_origin
        return max(4, math.hypot(x-cx, y-cy))

    def _build_pts(self, x, y):
        cx, cy = self.shape_origin; r = self._shape_r(x, y)
        if self.tool == TOOL_CIRCLE:   return make_circle(cx, cy, r)
        if self.tool == TOOL_SQUARE:   return make_square(cx, cy, r)
        if self.tool == TOOL_TRIANGLE: return make_triangle(cx, cy, r)
        if self.tool == TOOL_STAR:     return make_star(cx, cy, r)
        return []

    def _shape_preview(self, x, y):
        for cid in self.shape_ids: self.canvas.delete(cid)
        self.shape_ids = []
        pts = self._build_pts(x, y)
        if len(pts) >= 2:
            flat = [c for p in pts for c in p]
            self.shape_ids.append(
                self.canvas.create_line(*flat, fill=PREVIEW_CLR, width=1,
                                        dash=(4,3), tags="preview"))
        cx, cy = self.shape_origin; r = 3
        self.shape_ids.append(
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                                    fill=PREVIEW_CLR, outline="", tags="preview"))
        self.status_var.set(f"r = {self._shape_r(x,y):.0f} px")

    def _shape_commit(self, x, y):
        for cid in self.shape_ids: self.canvas.delete(cid)
        self.shape_ids = []
        pts = self._build_pts(x, y)
        if pts: self._commit_stroke(self.tool, pts)
        self.shape_origin = None
        self._on_tool_change()

    # ── Line tool ─────────────────────────────────────────────────────────────
    def _line_click(self, x, y):
        if self.line_start is None:
            self.line_start = (x, y)
            self.status_var.set("Click end point  ·  RMB cancel")
        else:
            x0, y0 = self.line_start
            self._commit_stroke(TOOL_LINE, [(x0,y0),(x,y)])
            if self.preview_id: self.canvas.delete(self.preview_id)
            self.preview_id = None; self.line_start = None
            self.status_var.set(TOOL_HINTS[TOOL_LINE])

    def _line_motion(self, x, y):
        if self.line_start is None: return
        if self.preview_id: self.canvas.delete(self.preview_id)
        x0, y0 = self.line_start
        self.preview_id = self.canvas.create_line(
            x0, y0, x, y, fill=PREVIEW_CLR, width=1, dash=(4,3), tags="preview")

    # ── Curve tool ────────────────────────────────────────────────────────────
    def _curve_click(self, x, y):
        if self.curve_step == 0:
            self.curve_start = (x,y); self.curve_step = 1
            self.status_var.set("Click end point")
        elif self.curve_step == 1:
            self.curve_end = (x,y)
            mx = (self.curve_start[0]+x)/2; my = (self.curve_start[1]+y)/2
            self.curve_ctrl = (mx,my); self.curve_step = 2
            self.status_var.set("Drag bend handle  ·  click to confirm")
        elif self.curve_step == 2:
            pts = bezier_points(self.curve_start, self.curve_ctrl, self.curve_end)
            self._commit_stroke(TOOL_CURVE, pts)
            for cid in self.curve_ids: self.canvas.delete(cid)
            self.curve_ids = []; self.curve_step = 0
            self.curve_start = self.curve_end = self.curve_ctrl = None
            self.status_var.set(TOOL_HINTS[TOOL_CURVE])

    def _curve_motion(self, x, y):
        if self.curve_step == 1 and self.curve_start:
            for cid in self.curve_ids: self.canvas.delete(cid)
            self.curve_ids = [self.canvas.create_line(
                self.curve_start[0], self.curve_start[1], x, y,
                fill=PREVIEW_CLR, width=1, dash=(4,3), tags="preview")]
        elif self.curve_step == 2:
            self.curve_ctrl = (x,y)
            pts = bezier_points(self.curve_start, self.curve_ctrl, self.curve_end)
            for cid in self.curve_ids: self.canvas.delete(cid)
            self.curve_ids = []
            flat = [c for p in pts for c in p]
            if len(flat) >= 4:
                self.curve_ids.append(self.canvas.create_line(
                    *flat, fill=PREVIEW_CLR, width=1, tags="preview"))
            r = 5
            self.curve_ids += [
                self.canvas.create_line(self.curve_start[0], self.curve_start[1],
                                        x, y, fill=TEXT2, dash=(2,4), tags="preview"),
                self.canvas.create_line(self.curve_end[0], self.curve_end[1],
                                        x, y, fill=TEXT2, dash=(2,4), tags="preview"),
                self.canvas.create_oval(x-r, y-r, x+r, y+r,
                                        fill=PREVIEW_CLR, outline="", tags="preview"),
            ]

    # ── Stroke management ─────────────────────────────────────────────────────
    def _commit_stroke(self, stype, points):
        flat = [c for p in points for c in p]
        if len(flat) >= 4:
            self.canvas.create_line(*flat, fill=STROKE_CLR,
                                    width=self.pen_width,
                                    capstyle=tk.ROUND, tags="stroke")
        self.strokes.append({"type": stype, "points": points})
        self._update_stats()

    def undo_stroke(self):
        if not self.strokes: return
        self._cancel_preview()
        self.strokes.pop()
        self.canvas.delete("stroke")
        for s in self.strokes:
            flat = [c for p in s["points"] for c in p]
            if len(flat) >= 4:
                self.canvas.create_line(*flat, fill=STROKE_CLR,
                                        width=self.pen_width,
                                        capstyle=tk.ROUND, tags="stroke")
        self._update_stats()
        self.status_var.set("Undo")

    def clear_canvas(self):
        if self.strokes and not messagebox.askyesno("Clear canvas",
                "Remove all strokes?", parent=self.root): return
        self._cancel_preview()
        self.strokes.clear()
        self.canvas.delete("stroke")
        self._update_stats()
        self.status_var.set("Canvas cleared")

    def _update_stats(self):
        n = len(self.strokes)
        self.strokes_var.set(f"{n} stroke{'s' if n!=1 else ''}")

    # ── Save G-code ───────────────────────────────────────────────────────────
    def save_gcode(self):
        if not self.strokes:
            messagebox.showwarning("Empty canvas", "Draw something first.", parent=self.root)
            return
        path = filedialog.asksaveasfilename(defaultextension=".gcode",
                                            filetypes=[("G-code","*.gcode")],
                                            initialfile="output.gcode")
        if not path: return
        with open(path, "w") as f:
            f.write(generate_gcode(self.strokes, feed_draw=self.feed_var.get()))
        self.status_var.set(f"Saved  {os.path.basename(path)}")

    # ── Calibration ───────────────────────────────────────────────────────────
    def open_calibrate(self):
        """Fixed calibration G-code — straight line along X axis and back."""

        gcode_lines = [
            "G28",
            "M3",
            "G1 X158.08 Y0.00 F600",
            "G1 X138.35 Y0.00 F600",
            "G1 X118.62 Y0.00 F600",
            "G1 X98.90  Y0.00 F600",
            "G1 X79.18  Y0.00 F600",
            "G1 X59.45  Y0.00 F600",
            "G1 X39.72  Y0.00 F600",
            "G1 X20.00  Y0.00 F600",
            "G1 X39.73  Y0.00 F600",
            "G1 X59.45  Y0.00 F600",
            "G1 X79.18  Y0.00 F600",
            "G1 X98.90  Y0.00 F600",
            "G1 X118.62 Y0.00 F600",
            "G1 X138.35 Y0.00 F600",
            "G1 X158.08 Y0.00 F600",
            "G1 X177.80 Y0.00 F600",
            "M5",
            "G28",
            "M0",
        ]

        lines = ['const char* GCODE[] = {']
        for l in gcode_lines:
            escaped = l.replace('"', '\\"')
            lines.append(f'    "{escaped}",')
        lines.append('    nullptr')
        lines.append('};')
        lines.append('const int GCODE_COUNT = sizeof(GCODE) / sizeof(GCODE[0]);')
        cpp = '\n'.join(lines)

        self.root.clipboard_clear()
        self.root.clipboard_append(cpp)
        self.root.update()

        win = _make_dialog(self.root, "Calibration G-code", "540x400")
        _dialog_title(win, "📋  CALIBRATION — PASTE INTO main.cpp")

        tk.Label(win,
                 text="Straight line along X axis and back.\n"
                      "If the line curves, adjust LINK1_MM / LINK2_MM in firmware.\n"
                      "Copied to clipboard!",
                 bg=BG2, fg=TEXT2, font=FONT_LABEL,
                 justify=tk.CENTER).pack(pady=(0, 8))

        txt = tk.Text(win, bg=BG, fg=ACCENT, font=FONT_MONO,
                      relief="flat", padx=8, pady=6, wrap=tk.NONE,
                      height=16, width=58)
        txt.pack(fill=tk.BOTH, expand=True, padx=16, pady=4)
        txt.insert("1.0", cpp)
        txt.config(state=tk.DISABLED)

        scroll = tk.Scrollbar(win, orient=tk.HORIZONTAL, command=txt.xview)
        scroll.pack(fill=tk.X, padx=16)
        txt.config(xscrollcommand=scroll.set)

        FlatButton(win, "📋  COPY AGAIN", lambda: (
            self.root.clipboard_clear(),
            self.root.clipboard_append(cpp),
            self.root.update()
        ), hover_bg=ACCENT, hover_fg=BG).pack(pady=8)

    # ── Copy for CPP ──────────────────────────────────────────────────────────
    def copy_for_cpp(self):
        """Generate the GCODE[] array ready to paste into main.cpp."""
        if not self.strokes:
            messagebox.showwarning("Empty canvas", "Draw something first.", parent=self.root)
            return

        gcode = generate_gcode(self.strokes, feed_draw=self.feed_var.get())
        cmd_lines = [l for l in gcode.splitlines()
                     if l.strip() and not l.strip().startswith(';')]

        lines = ['const char* GCODE[] = {']
        for l in cmd_lines:
            escaped = l.replace('"', '\\"')
            lines.append(f'    "{escaped}",')
        lines.append('    nullptr')
        lines.append('};')
        lines.append('const int GCODE_COUNT = sizeof(GCODE) / sizeof(GCODE[0]);')
        cpp = '\n'.join(lines)

        self.root.clipboard_clear()
        self.root.clipboard_append(cpp)
        self.root.update()

        win = _make_dialog(self.root, "Copy for main.cpp", "540x400")
        _dialog_title(win, "📋  PASTE INTO main.cpp")
        tk.Label(win,
                 text="Replace the GCODE[] array in main.cpp with this,\n"
                      "then flash via PlatformIO. Copied to clipboard!",
                 bg=BG2, fg=TEXT2, font=FONT_LABEL, justify=tk.CENTER).pack(pady=(0,8))

        txt = tk.Text(win, bg=BG, fg=ACCENT, font=FONT_MONO,
                      relief="flat", padx=8, pady=6, wrap=tk.NONE,
                      height=16, width=58)
        txt.pack(fill=tk.BOTH, expand=True, padx=16, pady=4)
        txt.insert("1.0", cpp)
        txt.config(state=tk.DISABLED)

        scroll = tk.Scrollbar(win, orient=tk.HORIZONTAL, command=txt.xview)
        scroll.pack(fill=tk.X, padx=16)
        txt.config(xscrollcommand=scroll.set)

        FlatButton(win, "📋  COPY AGAIN", lambda: (
            self.root.clipboard_clear(),
            self.root.clipboard_append(cpp),
            self.root.update()
        ), hover_bg=ACCENT, hover_fg=BG).pack(pady=8)


# ── Dialog helpers ────────────────────────────────────────────────────────────
def _make_dialog(parent, title, geometry):
    win = tk.Toplevel(parent)
    win.title(title)
    win.geometry(geometry)
    win.resizable(False, False)
    win.configure(bg=BG2)
    win.grab_set()
    return win

def _dialog_title(win, text):
    tk.Label(win, text=text, bg=BG2, fg=ACCENT,
             font=("Courier New", 13, "bold")).pack(pady=(16,6))

def _field_label(win, text, parent=None):
    target = parent or win
    tk.Label(target, text=text, bg=BG2, fg=TEXT2,
             font=FONT_SMALL, anchor="w").pack(
                 fill=tk.X if parent is None else tk.NONE,
                 side=tk.LEFT if parent else tk.TOP,
                 padx=0 if parent else 24)

def _entry(win, var, width=30):
    tk.Entry(win, textvariable=var, bg=BG3, fg=TEXT,
             insertbackground=ACCENT, relief="flat",
             font=FONT_MONO, width=width).pack(
                 anchor="w", padx=24, pady=(2,6))


# ── Entry ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    DrawingApp(root)
    root.mainloop()