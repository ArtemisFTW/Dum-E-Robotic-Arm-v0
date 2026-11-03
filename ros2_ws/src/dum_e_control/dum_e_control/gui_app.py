"""Original Arm Console layout, backed by ROS 2 services and topics."""

from datetime import datetime
import sys
from time import monotonic
import tkinter as tk
from tkinter import ttk

from .gui_model import (
    CARTESIAN_SLIDER_LIMITS, HOME_JOINTS, JOINT_LIMITS, InterfaceState, display, whole_degrees,
)
from .gui_model import CONFIG_PATH, conditioning
from .gui_transport import SerialStreamer, DRY_RUN_PORT
from .gui_pose_loop import LoopPlayer, make_pose


BG = "#101419"
PANEL = "#191F27"
FIELD = "#11171E"
EDGE = "#303945"
TEXT = "#EDF2F7"
MUTED = "#96A3B3"
ACCENT = "#B4F272"
BLUE = "#79B9FF"
ORANGE = "#F0B66D"
RED = "#FF8D91"


def label(parent, text="", size=10, color=TEXT, bold=False, **kwargs):
    return tk.Label(
        parent, text=text, bg=parent.cget("bg"), fg=color,
        font=("Segoe UI", size, "bold" if bold else "normal"), **kwargs,
    )


def button(parent, text, command, primary=False, **kwargs):
    return tk.Button(
        parent, text=text, command=command,
        bg=ACCENT if primary else EDGE, fg=BG if primary else TEXT,
        activebackground="#C9FF94" if primary else "#425063",
        activeforeground=BG if primary else TEXT,
        disabledforeground="#697584", relief="flat", bd=0,
        font=("Segoe UI", 10, "bold"), padx=16, pady=9,
        cursor="hand2", **kwargs,
    )


class ControlRow(tk.Frame):
    def __init__(self, parent, name, description, limits, value, unit, callback, color):
        super().__init__(parent, bg=PANEL)
        self.callback = callback
        self.integer = unit == "°"
        self.enabled = True
        self.value = whole_degrees(value) if self.integer else float(value)
        self.variable = tk.IntVar(value=self.value) if self.integer else tk.DoubleVar(value=value)
        self.text = tk.StringVar(value=self._format(value))
        self.columnconfigure(1, weight=1)
        self.name_label = label(self, name, 12, color, True, width=3, anchor="w")
        self.name_label.grid(row=0, column=0, rowspan=2, sticky="nw", padx=(0, 12))
        label(self, description, 9, MUTED).grid(row=0, column=1, sticky="w")
        self.entry = tk.Entry(
            self, textvariable=self.text, width=7, justify="right",
            bg=FIELD, fg=TEXT, insertbackground=ACCENT, relief="flat",
            highlightthickness=1, highlightbackground=EDGE, highlightcolor=ACCENT,
            disabledbackground=PANEL, disabledforeground="#627080",
            font=("Consolas", 13),
        )
        self.entry.grid(row=0, column=2, rowspan=2, ipady=7, sticky="n", padx=(14, 6))
        label(self, unit, 10, MUTED).grid(row=0, column=3, rowspan=2, sticky="n", pady=8)
        self.scale = ttk.Scale(
            self, from_=limits[0], to=limits[1], variable=self.variable,
            command=self._slide, style="Arm.Horizontal.TScale", takefocus=True,
        )
        self.scale.grid(row=1, column=1, sticky="ew", pady=(5, 0))
        ticks = tk.Frame(self, bg=PANEL)
        ticks.grid(row=2, column=1, sticky="ew")
        label(ticks, display(limits[0]), 8, MUTED).pack(side="left")
        label(ticks, display(limits[1]), 8, MUTED).pack(side="right")
        self.entry.bind("<Return>", self._commit)
        self.entry.bind("<FocusOut>", self._commit)
        self.entry.bind("<Escape>", lambda _event: self.sync(self.value))
        self.scale.bind("<Left>", lambda event: self._step(-1))
        self.scale.bind("<Right>", lambda event: self._step(1))

    def _step(self, step):
        if self.enabled:
            value = min(float(self.scale.cget("to")), max(float(self.scale.cget("from")), self.value + step))
            self.callback(str(value))
        return "break"

    def _slide(self, value):
        if self.enabled:
            self.callback(str(whole_degrees(float(value)) if self.integer else round(float(value), 1)))

    def _format(self, value):
        return str(whole_degrees(value)) if self.integer else display(value)

    def _commit(self, _event=None):
        # Readouts may display fewer decimals than FK stores internally. Merely
        # tabbing through a box must not round/re-solve a previously valid pose.
        if self.enabled and self.text.get() != self._format(self.value):
            self.callback(self.text.get())
        return "break"

    def sync(self, value):
        self.value = value
        self.variable.set(value)
        self.text.set(self._format(value))

    def enable(self, enabled):
        self.enabled = enabled
        state = "normal" if enabled else "disabled"
        self.scale.configure(state=state)
        self.entry.configure(state=state)


class ArmConsole(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Arm Console — ROS 2 Control")
        self.configure(bg=BG)
        self.geometry("1240x810")
        self.minsize(1060, 770)
        self.state_model = InterfaceState()
        self.toast_job = None
        self.joint_rows = []
        self.cartesian_rows = []
        self.mode_buttons = {}
        self.streamer = SerialStreamer()
        self.loop_player = LoopPlayer()
        self.poses = []
        self.page = "Control"
        self.transport_job = None
        self._styles()
        self.viewport = tk.Canvas(self, bg=BG, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.viewport.yview)
        self.viewport.configure(yscrollcommand=self.scrollbar.set)
        self.viewport.grid(row=0, column=0, sticky="nsew")
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.body = tk.Frame(self.viewport, bg=BG)
        self.status_area = tk.Frame(self, bg=BG)
        self.status_area.grid(row=1, column=0, columnspan=2, sticky="ew")
        self.body_window = self.viewport.create_window(0, 0, window=self.body, anchor="nw")
        self.body.bind("<Configure>", self._layout_body)
        self.viewport.bind("<Configure>", self._layout_body)
        self.bind("<MouseWheel>", self._scroll)
        self.bind("<Button-4>", lambda _event: self.viewport.yview_scroll(-1, "units"))
        self.bind("<Button-5>", lambda _event: self.viewport.yview_scroll(1, "units"))
        self._build()
        self._fit_window()
        self._set_mode("Jog", announce=False)
        self._refresh_preview()
        self._refresh_ports()
        self.port.set(self.streamer.selected_port or DRY_RUN_PORT)
        self._log("Interface ready. Select the ESP32 port (or ROS 2 dry run), then Connect & stream.")
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.transport_job = self.after(50, self._poll_transport)
        self.bind("<Control-Key-1>", lambda _event: self._set_mode("Jog"))
        self.bind("<Control-Key-2>", lambda _event: self._set_mode("FK"))
        self.bind("<Control-Key-3>", lambda _event: self._set_mode("IK"))

    def _fit_window(self):
        # Tk scales point-sized fonts with display DPI. Scale the window too so
        # the bottom message and footer remain visible on Windows at 125/150%.
        self.update_idletasks()
        dpi_scale = self.winfo_fpixels("1i") / 96
        width = min(round(1240 * dpi_scale), self.winfo_screenwidth() - 80)
        height = min(max(round(780 * dpi_scale), self.body.winfo_reqheight() + self.status_area.winfo_reqheight() + 8),
                     self.winfo_screenheight() - 90)
        self.minsize(min(round(1060 * dpi_scale), width),
                     min(round(740 * dpi_scale), height))
        left = max(0, (self.winfo_screenwidth() - width) // 2)
        top = max(0, (self.winfo_screenheight() - height - 40) // 2)
        self.geometry(f"{width}x{height}+{left}+{top}")

    def _layout_body(self, _event=None):
        width, height = self.viewport.winfo_width(), self.viewport.winfo_height()
        needed = self.body.winfo_reqheight()
        self.viewport.itemconfigure(self.body_window, width=width, height=max(height, needed))
        self.viewport.configure(scrollregion=(0, 0, width, max(height, needed)))
        if needed > height:
            self.scrollbar.grid(row=0, column=1, sticky="ns")
        else:
            self.scrollbar.grid_remove()
            self.viewport.yview_moveto(0)

    def _scroll(self, event):
        if self.body.winfo_reqheight() > self.viewport.winfo_height():
            self.viewport.yview_scroll(-int(event.delta/120), "units")

    def _styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Arm.Horizontal.TScale", background=PANEL, troughcolor=FIELD,
                        bordercolor=FIELD, lightcolor=ACCENT, darkcolor=ACCENT,
                        sliderlength=20, sliderthickness=15)
        style.map("Arm.Horizontal.TScale", background=[("disabled", EDGE), ("!disabled", ACCENT)])
        style.configure("Arm.TCombobox", fieldbackground=FIELD, background=EDGE,
                        foreground=TEXT, arrowcolor=MUTED, bordercolor=EDGE, padding=7)
        style.map("Arm.TCombobox", fieldbackground=[("readonly", FIELD)],
                  foreground=[("readonly", TEXT)], selectbackground=[("readonly", FIELD)],
                  selectforeground=[("readonly", TEXT)])
        self.option_add("*TCombobox*Listbox.background", FIELD)
        self.option_add("*TCombobox*Listbox.foreground", TEXT)
        self.option_add("*TCombobox*Listbox.selectBackground", EDGE)
        style.configure("Arm.Treeview", background=FIELD, fieldbackground=FIELD,
                        foreground=TEXT, rowheight=29, bordercolor=EDGE, font=("Consolas", 10))
        style.configure("Arm.Treeview.Heading", background=EDGE, foreground=TEXT,
                        font=("Segoe UI", 10, "bold"), padding=6)
        style.map("Arm.Treeview", background=[("selected", "#344D30")], foreground=[("selected", ACCENT)])

    def _card(self, parent, column, row=0):
        card = tk.Frame(parent, bg=PANEL, highlightbackground=EDGE, highlightthickness=1)
        card.grid(row=row, column=column, sticky="nsew", padx=(0, 12) if column == 0 else (0, 0))
        return card

    def _build(self):
        header = tk.Frame(self.body, bg=BG)
        header.pack(fill="x", padx=28, pady=(22, 0))
        mark = tk.Canvas(header, width=40, height=40, bg=BG, highlightthickness=0)
        mark.pack(side="left", padx=(0, 12))
        mark.create_line(5, 34, 13, 14, 29, 7, 36, 18, fill=ACCENT, width=4)
        for x, y in ((5, 34), (13, 14), (29, 7), (36, 18)):
            mark.create_oval(x-3, y-3, x+3, y+3, fill=BG, outline=ACCENT, width=2)
        branding = tk.Frame(header, bg=BG)
        branding.pack(side="left")
        label(branding, "Arm Console", 23, bold=True).pack(anchor="w")
        label(branding, "3-axis control workspace  /  ROS 2 Jazzy", 10, MUTED).pack(anchor="w")
        self.connection_badge = label(header, "●  UART OFFLINE", 10, MUTED, True, padx=14, pady=9,
                                      highlightbackground=EDGE, highlightthickness=1)
        self.connection_badge.pack(side="right")

        navigation = tk.Frame(self.body, bg=BG)
        navigation.pack(fill="x", padx=28, pady=(18, 0))
        self.page_buttons = {}
        for name in ("Control", "Loop"):
            item = button(navigation, name, lambda name=name: self._show_page(name), primary=name == "Control")
            item.pack(side="left", padx=(0, 8))
            self.page_buttons[name] = item
        self.control_page = tk.Frame(self.body, bg=BG)
        self.control_page.pack(fill="both", expand=True)
        self.loop_page = tk.Frame(self.body, bg=BG)
        self._build_loop_page()

        toolbar = tk.Frame(self.control_page, bg=BG)
        toolbar.pack(fill="x", padx=28, pady=(24, 18))
        for title, subtitle in (("Jog", "DIRECT"), ("FK", "JOINT → XYZ"), ("IK", "XYZ → JOINT")):
            item = button(toolbar, f"{title}   /   {subtitle}", lambda m=title: self._set_mode(m))
            item.pack(side="left", padx=(0, 8))
            self.mode_buttons[title] = item
        self.pause_button = button(toolbar, "Pause input", self._toggle_pause)
        self.pause_button.pack(side="right")
        button(toolbar, "Model / mapping", self._show_model).pack(side="right", padx=(0, 8))

        self.mode_title = label(self.control_page, "", 10, MUTED, anchor="w")
        self.mode_title.pack(fill="x", padx=30, pady=(0, 12))

        controls = tk.Frame(self.control_page, bg=BG)
        controls.pack(fill="both", expand=True, padx=28)
        controls.columnconfigure((0, 1), weight=1, uniform="controls")
        controls.rowconfigure(0, weight=1)
        joint_card = self._card(controls, 0)
        target_card = self._card(controls, 1)

        top = tk.Frame(joint_card, bg=PANEL)
        top.pack(fill="x", padx=20, pady=(18, 0))
        label(top, "Joint setpoints", 15, bold=True).pack(side="left")
        self.joint_badge = label(top, "EDITABLE", 9, ACCENT, True)
        self.joint_badge.pack(side="right")
        label(joint_card, "All servos: 0–180° · J1 reversed · J2: 0° level / 90° upright", 9, MUTED).pack(anchor="w", padx=20, pady=(4, 12))
        for index, (limits, description) in enumerate(zip(JOINT_LIMITS, ("Base", "Shoulder", "Elbow"))):
            row = ControlRow(joint_card, f"J{index+1}", description, limits, HOME_JOINTS[index], "°",
                             lambda value, i=index: self._joint_changed(i, value), ACCENT)
            row.pack(fill="x", padx=20, pady=(0, 12))
            self.joint_rows.append(row)
        self.dh_label = label(joint_card, "", 9, MUTED, anchor="w")
        self.dh_label.pack(fill="x", padx=20, pady=(0, 12))

        top = tk.Frame(target_card, bg=PANEL)
        top.pack(fill="x", padx=20, pady=(18, 0))
        self.target_title = label(top, "Cartesian position", 15, bold=True)
        self.target_title.pack(side="left")
        self.target_badge = label(top, "LOCKED", 9, MUTED, True)
        self.target_badge.pack(side="right")
        self.target_description = label(target_card, "", 9, MUTED)
        self.target_description.pack(anchor="w", padx=20, pady=(4, 12))
        for index, (name, description, color) in enumerate(zip("XYZ", ("X coordinate", "Y coordinate", "Z coordinate"), (RED, ACCENT, BLUE))):
            row = ControlRow(target_card, name, description, CARTESIAN_SLIDER_LIMITS, 0, "mm",
                             lambda value, i=index: self._target_changed(i, value), color)
            row.pack(fill="x", padx=20, pady=(0, 12))
            self.cartesian_rows.append(row)
        target_actions = tk.Frame(target_card, bg=PANEL)
        target_actions.pack(fill="x", padx=20, pady=(0, 12))
        self.apply_button = button(target_actions, "Solve XYZ", self._apply_target)
        self.apply_button.pack(side="left")
        self.use_pose_button = button(target_actions, "Use current pose", self._use_current_pose)
        self.use_pose_button.pack(side="left", padx=(8, 0))

        self.lower = tk.Frame(self.body, bg=BG)
        self.lower.pack(fill="x", padx=28, pady=(14, 0))
        self.lower.columnconfigure((0, 1), weight=1, uniform="lower")
        uart = self._card(self.lower, 0)
        packet = self._card(self.lower, 1)
        inner = tk.Frame(uart, bg=PANEL)
        inner.pack(fill="both", padx=20, pady=16)
        inner.columnconfigure(0, weight=3)
        inner.columnconfigure(1, weight=2)
        label(inner, "UART connection", 13, bold=True).grid(row=0, column=0, sticky="w")
        self.uart_status = label(inner, "●  OFFLINE", 9, MUTED, True)
        self.uart_status.grid(row=0, column=1, sticky="e")
        label(inner, "Port", 9, MUTED).grid(row=1, column=0, sticky="w", pady=(12, 3))
        label(inner, "Baud rate", 9, MUTED).grid(row=1, column=1, sticky="w", padx=(10, 0), pady=(12, 3))
        self.port = tk.StringVar(value="Select a port")
        self.port_combo = ttk.Combobox(inner, textvariable=self.port, state="readonly", style="Arm.TCombobox", width=18)
        self.port_combo.grid(row=2, column=0, sticky="ew")
        self.baud = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(inner, textvariable=self.baud, state="readonly", width=10,
                                      style="Arm.TCombobox", values=("115200",))
        self.baud_combo.grid(row=2, column=1, sticky="ew", padx=(10, 0))
        actions = tk.Frame(inner, bg=PANEL)
        actions.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        self.refresh_button = button(actions, "Refresh ports", self._refresh_ports)
        self.refresh_button.pack(side="left")
        self.connect_button = button(actions, "Connect & stream", self._toggle_connection)
        self.connect_button.pack(side="left", padx=(8, 0))
        self.ack_label = label(inner, "Connect sends the shown pose after a 2.5-second startup wait.", 9, MUTED)
        self.ack_label.grid(row=4, column=0, columnspan=2, sticky="w", pady=(10, 0))

        inner = tk.Frame(packet, bg=PANEL)
        inner.pack(fill="both", padx=20, pady=16)
        top = tk.Frame(inner, bg=PANEL)
        top.pack(fill="x")
        label(top, "Servo command / UART", 13, bold=True).pack(side="left")
        label(top, "J1,J2,J3\\n", 9, MUTED).pack(side="right")
        self.packet_label = label(inner, self.state_model.packet.rstrip("\n"), 27, ACCENT, True, anchor="w")
        self.packet_label.configure(font=("Consolas", 27, "bold"))
        self.packet_label.pack(fill="x", pady=(12, 2))
        self.packet_note = label(inner, "", 9, MUTED, anchor="w")
        self.packet_note.pack(fill="x")
        label(inner, "Accepted DH angles → integer servo commands", 9, MUTED).pack(anchor="w", pady=(12, 4))
        self.calculation_labels = []
        for _ in range(3):
            calculation = label(inner, "", 11, TEXT, anchor="w")
            calculation.configure(font=("Consolas", 11))
            calculation.pack(fill="x", pady=2)
            self.calculation_labels.append(calculation)
        self.last_sent_label = label(inner, "Last UART write: none", 9, MUTED, anchor="w")
        self.last_sent_label.configure(font=("Consolas", 9))
        self.last_sent_label.pack(fill="x", pady=(10, 0))
        actions = tk.Frame(inner, bg=PANEL)
        actions.pack(fill="x", pady=(12, 0))
        button(actions, "Copy frame", self._copy_packet).pack(side="left")
        button(actions, "Activity", self._show_activity).pack(side="left", padx=(8, 0))
        self.revision_label = label(actions, "0 edits · 0 sent", 9, MUTED)
        self.revision_label.pack(side="right")

        self.banner = tk.Frame(self.status_area, bg="#242A22", highlightthickness=1, highlightbackground="#3D4B34")
        self.banner.pack(fill="x", padx=28, pady=(14, 0))
        self.banner_label = label(self.banner, "", 10, ACCENT, anchor="w", padx=14, pady=11)
        self.banner_label.pack(fill="x")
        self.banner.bind("<Configure>", lambda event: self.banner_label.configure(wraplength=max(200, event.width-32)))
        footer = tk.Frame(self.status_area, bg=BG)
        footer.pack(fill="x", padx=30, pady=(10, 12))
        self.link_footer = label(footer, "UART offline · 115200 baud · 8N1 · 20 Hz when connected", 9, MUTED)
        self.link_footer.pack(side="left")
        label(footer, "Ctrl+1 Jog   ·   Ctrl+2 FK   ·   Ctrl+3 IK", 9, MUTED).pack(side="right")
        self.events = []

    def _build_loop_page(self):
        label(self.loop_page, "Loop / pose sequence", 18, bold=True).pack(anchor="w", padx=28, pady=(20, 4))
        label(self.loop_page, "Add joint or XYZ poses. Send loop repeats the list, holding each pose for 2 seconds after PWM arrival.",
              10, MUTED).pack(anchor="w", padx=28, pady=(0, 12))
        form = tk.Frame(self.loop_page, bg=PANEL, padx=18, pady=16)
        form.pack(fill="x", padx=28)
        self.pose_source = tk.StringVar(value="Joints")
        self.pose_source_combo = ttk.Combobox(form, textvariable=self.pose_source, values=("Joints", "XYZ"),
                                              state="readonly", style="Arm.TCombobox", width=10)
        self.pose_source_combo.grid(row=1, column=0, sticky="w", padx=(0, 20))
        self.pose_source_combo.bind("<<ComboboxSelected>>", lambda _event: self._pose_input_changed())
        label(form, "Input type", 9, MUTED).grid(row=0, column=0, sticky="w", pady=(0, 5))
        self.pose_fields, self.pose_entries, self.pose_field_labels = [], [], []
        for index, value in enumerate(HOME_JOINTS):
            form.columnconfigure(index+1, weight=1)
            caption = label(form, f"J{index+1} (°)", 9, MUTED)
            caption.grid(row=0, column=index+1, sticky="w", pady=(0, 5), padx=(0, 15))
            variable = tk.StringVar(value=str(value))
            entry = tk.Entry(form, textvariable=variable, bg=FIELD, fg=TEXT, insertbackground=ACCENT,
                             relief="flat", font=("Consolas", 13), width=12)
            entry.grid(row=1, column=index+1, sticky="ew", ipady=8, padx=(0, 15))
            self.pose_fields.append(variable)
            self.pose_entries.append(entry)
            self.pose_field_labels.append(caption)
        actions = tk.Frame(form, bg=PANEL)
        actions.grid(row=2, column=0, columnspan=4, sticky="w", pady=(12, 0))
        self.pose_edit_buttons = []
        for text, callback in (("Add pose", self._add_pose), ("Update selected", self._update_pose),
                               ("Use current command", self._pose_use_current)):
            item = button(actions, text, callback)
            item.pack(side="left", padx=(0, 8))
            self.pose_edit_buttons.append(item)
        listing = tk.Frame(self.loop_page, bg=BG)
        listing.pack(fill="both", expand=True, padx=28, pady=(14, 0))
        columns = ("order", "source", "input", "j1", "j2", "j3", "xyz")
        self.pose_tree = ttk.Treeview(listing, columns=columns, show="headings", height=8,
                                     selectmode="browse", style="Arm.Treeview")
        for name, title, width in zip(columns, ("#", "Type", "Entered pose", "J1°", "J2°", "J3°", "Target FK XYZ / mm"),
                                       (35, 65, 220, 55, 55, 55, 260)):
            self.pose_tree.heading(name, text=title)
            self.pose_tree.column(name, width=width, minwidth=width, stretch=name in ("input", "xyz"))
        self.pose_tree.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(listing, orient="vertical", command=self.pose_tree.yview)
        scroll.pack(side="right", fill="y")
        self.pose_tree.configure(yscrollcommand=scroll.set)
        self.pose_tree.bind("<<TreeviewSelect>>", self._pose_selected)
        actions = tk.Frame(self.loop_page, bg=BG)
        actions.pack(fill="x", padx=28, pady=(10, 0))
        for text, callback in (("Remove", self._remove_pose), ("Move up", lambda: self._move_pose(-1)),
                               ("Move down", lambda: self._move_pose(1)), ("Clear list", self._clear_poses)):
            item = button(actions, text, callback)
            item.pack(side="left", padx=(0, 8))
            self.pose_edit_buttons.append(item)
        self.loop_send_button = button(actions, "Send loop", self._start_loop, primary=True)
        self.loop_send_button.pack(side="right")
        self.loop_stop_button = button(actions, "Stop loop", self._stop_loop, state="disabled")
        self.loop_stop_button.pack(side="right", padx=(8, 8))
        self.loop_status = label(self.loop_page, self.loop_player.description(monotonic()), 11, ACCENT, anchor="w")
        self.loop_status.pack(fill="x", padx=30, pady=(12, 4))
        label(self.loop_page, "Editing the list sends nothing. Stop loop holds the current target. DONE is commanded PWM arrival, not measured position.",
              9, MUTED).pack(anchor="w", padx=30, pady=(0, 4))

    def _show_page(self, name):
        self.page = name
        self.control_page.pack_forget()
        self.loop_page.pack_forget()
        page = self.control_page if name == "Control" else self.loop_page
        page.pack(fill="both", expand=True, before=self.lower)
        for title, widget in self.page_buttons.items():
            widget.configure(bg=ACCENT if title == name else EDGE, fg=BG if title == name else TEXT)
        self.viewport.yview_moveto(0)

    def _pose_input_changed(self):
        names = ("J1 (°)", "J2 (°)", "J3 (°)") if self.pose_source.get() == "Joints" else ("X (mm)", "Y (mm)", "Z (mm)")
        for widget, name in zip(self.pose_field_labels, names):
            widget.configure(text=name)

    def _pose_use_current(self):
        if self.loop_player.running:
            return
        values = self.state_model.joints if self.pose_source.get() == "Joints" else self.state_model.position
        for variable, value in zip(self.pose_fields, values):
            variable.set(str(value))

    def _selected_pose_index(self):
        selection = self.pose_tree.selection()
        if not selection:
            raise ValueError("Select a pose in the list first.")
        return int(selection[0])

    def _pose_selected(self, _event=None):
        if self.loop_player.running or not self.pose_tree.selection():
            return
        pose = self.poses[self._selected_pose_index()]
        self.pose_source.set(pose.source)
        self._pose_input_changed()
        for variable, value in zip(self.pose_fields, pose.values):
            variable.set(str(value))

    def _read_pose(self, reference):
        return make_pose(self.pose_source.get(), [variable.get() for variable in self.pose_fields],
                         self.state_model.robot, reference)

    def _add_pose(self):
        if self.loop_player.running:
            return
        try:
            reference = self.poses[-1].joints if self.poses else self.state_model.joints
            pose = self._read_pose(reference)
        except ValueError as error:
            self._notify(f"Pose not added: {error} Last command held.", "error", temporary=False)
            return
        self.poses.append(pose)
        self._refresh_pose_list(len(self.poses)-1)
        self._notify(f"Pose {len(self.poses)} added. The arm has not been commanded.")

    def _update_pose(self):
        if self.loop_player.running:
            return
        try:
            index = self._selected_pose_index()
            self.poses[index] = self._read_pose(self.poses[index].joints)
            self._refresh_pose_list(index)
        except ValueError as error:
            self._notify(str(error), "error")

    def _refresh_pose_list(self, selected=None):
        self.pose_tree.delete(*self.pose_tree.get_children())
        for index, pose in enumerate(self.poses):
            entered = ", ".join(str(value) if pose.source == "Joints" else f"{value:.3f}" for value in pose.values)
            xyz = ", ".join(f"{value:.2f}" for value in pose.xyz)
            self.pose_tree.insert("", "end", iid=str(index), values=(index+1, pose.source, entered, *pose.joints, xyz))
        if selected is not None and self.poses:
            item = str(min(selected, len(self.poses)-1))
            self.pose_tree.selection_set(item)
            self.pose_tree.see(item)

    def _remove_pose(self):
        if self.loop_player.running:
            return
        try:
            index = self._selected_pose_index()
            del self.poses[index]
            self._refresh_pose_list(index)
        except ValueError as error:
            self._notify(str(error), "error")

    def _move_pose(self, direction):
        if self.loop_player.running:
            return
        try:
            index = self._selected_pose_index()
            other = index+direction
            if 0 <= other < len(self.poses):
                self.poses[index], self.poses[other] = self.poses[other], self.poses[index]
                self._refresh_pose_list(other)
        except ValueError as error:
            self._notify(str(error), "error")

    def _clear_poses(self):
        if not self.loop_player.running:
            self.poses.clear()
            self._refresh_pose_list()

    def _send_loop_pose(self, pose):
        self.state_model.accept_command(pose.joints)
        self._refresh_preview()
        self.pose_tree.selection_set(str(self.loop_player.index))
        self.pose_tree.see(str(self.loop_player.index))
        self._log(f"Loop pose {self.loop_player.index+1}: {pose.packet.rstrip()}")

    def _start_loop(self):
        if self.loop_player.running:
            return
        try:
            if self.state_model.paused:
                raise ValueError("Resume input before starting the loop.")
            # Revalidate every pose before replacing a single actuator command.
            poses = [make_pose(pose.source, pose.values, self.state_model.robot, pose.joints) for pose in self.poses]
            first = self.loop_player.start(poses, self.streamer.status(), monotonic())
            self._send_loop_pose(first)
        except ValueError as error:
            self.loop_player.stop()
            self._notify(str(error), "error", temporary=False)
        self._apply_locks()
        if self.loop_player.running:
            self._notify("Loop running. Each target is held for 2 seconds after the ESP32 reports PWM arrival.", temporary=False)

    def _stop_loop(self):
        was_running = self.loop_player.running
        self.loop_player.stop()
        self._apply_locks()
        self.loop_status.configure(text=self.loop_player.description(monotonic()))
        if was_running:
            self._notify("Loop stopped. The current target continues streaming; the servos may finish moving to it.", temporary=False)

    def _poll_loop(self, status):
        try:
            pose = self.loop_player.tick(status, monotonic())
            if pose is not None:
                self._send_loop_pose(pose)
        except ValueError as error:
            self._apply_locks()
            self._notify(str(error), "error", temporary=False)
        self.loop_status.configure(text=self.loop_player.description(monotonic()))

    def _set_mode(self, mode, announce=True):
        if self.loop_player.running:
            self._notify("Stop loop before changing manual modes.", "warning")
            return
        # Commit the active editor before changing which bank accepts inputs.
        focused = self.focus_get()
        for row in self.joint_rows + self.cartesian_rows:
            if focused is row.entry:
                row._commit()
        self.state_model.set_mode(mode)
        self._apply_locks()
        self._refresh_preview()
        if announce:
            self._log(f"Switched to {mode} mode.")
        self._baseline_message()

    def _apply_locks(self):
        mode = self.state_model.mode
        paused = self.state_model.paused or self.loop_player.running
        for widget in self.mode_buttons.values():
            widget.configure(state="disabled" if self.loop_player.running else "normal")
        editing = not self.loop_player.running
        self.pose_source_combo.configure(state="readonly" if editing else "disabled")
        for widget in self.pose_entries + self.pose_edit_buttons:
            widget.configure(state="normal" if editing else "disabled")
        self.loop_send_button.configure(state="normal" if editing else "disabled")
        self.loop_stop_button.configure(state="normal" if self.loop_player.running else "disabled")
        for name, widget in self.mode_buttons.items():
            selected = name == mode
            widget.configure(bg=ACCENT if selected else EDGE, fg=BG if selected else MUTED)
        for row in self.joint_rows:
            row.enable(mode != "IK" and not paused)
        for row in self.cartesian_rows:
            row.enable(mode == "IK" and not paused)
        self.joint_badge.configure(text="EDITABLE" if mode != "IK" and not paused else "LOCKED", fg=ACCENT if mode != "IK" and not paused else MUTED)
        self.target_badge.configure(text="EDITABLE" if mode == "IK" and not paused else "LOCKED", fg=BLUE if mode == "IK" and not paused else MUTED)
        descriptions = {
            "Jog": "Jog mode   /   Adjust joints directly. Connected servos follow sliders live at 20 Hz.",
            "FK": "Forward kinematics   /   Servo commands → calibrated DH angles → Cartesian position in mm.",
            "IK": "Inverse kinematics   /   Edit XYZ, then press Solve XYZ to move. Sliders only prepare a target.",
        }
        title = ("Loop running   /   Manual controls are locked. Use Stop loop to resume manual input." if self.loop_player.running else
                 "Input paused   /   All setpoints are locked." if paused else descriptions[mode])
        self.mode_title.configure(text=title)
        for widget in (self.apply_button, self.use_pose_button):
            widget.configure(state="normal" if mode == "IK" and not paused else "disabled")
        self.target_title.configure(text="Cartesian target" if mode == "IK" else "Cartesian position / FK")
        self.target_description.configure(text="App XYZ frame · reach and limits checked by IK" if mode == "IK" else "App XYZ frame: [DH X, DH Y, DH Z] · Z upward")

    def _joint_changed(self, index, text):
        if self.loop_player.running:
            return
        try:
            self.state_model.set_joint(index, text)
        except ValueError as error:
            self.joint_rows[index].sync(self.state_model.joints[index])
            self._notify(f"{error} Keeping the last accepted servo command.", "error")
            return
        self.joint_rows[index].sync(self.state_model.joints[index])
        self._refresh_preview()
        self._baseline_message()

    def _target_changed(self, index, text):
        if self.loop_player.running:
            return
        try:
            self.state_model.set_target(index, text)
        except ValueError as error:
            self._refresh_preview()
            self._notify(f"{error} Holding the last accepted servo command.", "error", temporary=False)
            return
        self._refresh_preview()
        self._baseline_message()

    def _apply_target(self):
        if self.loop_player.running:
            return
        try:
            self.state_model.set_target_vector([row.text.get() for row in self.cartesian_rows])
        except ValueError as error:
            self._refresh_preview()
            self._notify(f"{error} Holding the last accepted servo command.", "error", temporary=False)
            return
        self._refresh_preview()
        self._baseline_message()

    def _use_current_pose(self):
        if self.loop_player.running:
            return
        self.state_model.target = list(self.state_model.position)
        self.state_model.ik_error = self.state_model.ik_error_code = ""
        self.state_model.branch = ""
        self.state_model.target_dirty = False
        self._refresh_preview()
        self._baseline_message()

    def _refresh_preview(self):
        for row, value in zip(self.joint_rows, self.state_model.joints):
            row.sync(value)
        coords = self.state_model.target if self.state_model.mode == "IK" else self.state_model.position
        for row, value in zip(self.cartesian_rows, coords):
            row.sync(value)
        self.packet_label.configure(text=self.state_model.packet.rstrip("\n"))
        self.streamer.publish(self.state_model.packet)
        xyz = ", ".join(f"{value:.2f}" for value in self.state_model.position)
        self.packet_note.configure(text=f"Achieved app XYZ: ({xyz}) mm", justify="left")
        robot = self.state_model.robot
        # Reconstruct from accepted integer commands, so a rejected IK target
        # cannot appear here as an angle being sent. No second inversion on UART.
        for index, (widget, theta, offset, direction, command) in enumerate(zip(
                self.calculation_labels, self.state_model.dh_angles, robot.offsets,
                robot.directions, self.state_model.joints), start=1):
            sign = "−" if direction < 0 else "+"
            theta = 0 if theta == 0 else theta  # Avoid displaying negative zero.
            widget.configure(text=f"J{index} = round({offset:g} {sign} ({theta:g})) = {command}°")
        angles = ", ".join(str(whole_degrees(value)) for value in self.state_model.dh_angles)
        self.dh_label.configure(text=f"DH θ1, θ2, θ3:  ({angles})°")
        self.revision_label.configure(text=f"{self.state_model.revision} edits · {self.streamer.status().sent} sent")

    def _baseline_message(self):
        if self.loop_player.running:
            self._notify(self.loop_player.description(monotonic()), log=False, temporary=False)
        elif self.state_model.paused:
            self._notify("Input paused. A connected ESP32 continues receiving the last accepted command to hold position.", "warning", log=False, temporary=False)
        elif self.state_model.mode == "IK":
            if self.state_model.ik_error:
                self._notify(f"{self.state_model.ik_error_code}: {self.state_model.ik_error} Last command held.", "error", log=False, temporary=False)
            elif self.state_model.target_dirty:
                self._notify("Target edited. Press Solve XYZ to move. Holding the last accepted servo command.", log=False, temporary=False)
            elif self.state_model.branch:
                self._notify(f"IK solved · {self.state_model.branch} · rounded-command error {self.state_model.position_error:.3f} mm.", log=False, temporary=False)
            else:
                self._notify("IK ready. Target starts at the current FK position; commands change only after a valid solve.", log=False, temporary=False)
        else:
            _, singular, near = conditioning(self.state_model.dh_angles, self.state_model.robot)
            if singular or near:
                self._notify("Current FK pose is singular or near-singular. Manual control is available; IK rejects these poses.", "warning", log=False, temporary=False)
            else:
                self._notify("FK updated. Servo commands are within physical limits; connected UART streams the shown command.", log=False, temporary=False)

    def _notify(self, message, kind="info", log=True, temporary=True):
        if self.toast_job is not None:
            self.after_cancel(self.toast_job)
            self.toast_job = None
        color, background, prefix = {
            "error": (RED, "#352329", "INPUT ERROR"),
            "warning": (ORANGE, "#31291F", "HOLD"),
            "info": (ACCENT, "#242A22", "STATUS"),
        }[kind]
        self.banner.configure(bg=background, highlightbackground=color)
        self.banner_label.configure(text=f"{prefix}   /   {message}", bg=background, fg=color)
        if log:
            self._log(message)
        if temporary:
            self.toast_job = self.after(6500, self._baseline_message)

    def _toggle_pause(self):
        if self.loop_player.running:
            self._stop_loop()
        focused = self.focus_get()
        for row in self.joint_rows + self.cartesian_rows:
            if focused is row.entry:
                row._commit()
        self.state_model.paused = not self.state_model.paused
        self.pause_button.configure(text="Resume input" if self.state_model.paused else "Pause input")
        self._apply_locks()
        self._baseline_message()
        self._log("Inputs paused." if self.state_model.paused else "Inputs resumed.")

    def _refresh_ports(self):
        try:
            from serial.tools import list_ports
            devices = [DRY_RUN_PORT] + [port.device for port in sorted(list_ports.comports(), key=lambda p: (p.vid is None, p.device))]
            configured = self.streamer.selected_port
            if configured and configured not in devices:
                devices.insert(1, configured)
        except ImportError:
            self._notify("Install requirements.txt to enable pyserial port listing and UART streaming.", "warning")
            return
        except Exception as error:
            self._notify(f"Could not list serial ports: {error}", "error")
            return
        self.port_combo.configure(values=devices)
        if self.port.get() not in devices:
            self.port.set("Select a port" if devices else "No ports found")
        self._notify(f"Found {len(devices)} serial port(s). Port listing does not open a connection.")

    def _toggle_connection(self):
        if self.streamer.running:
            self._stop_loop()
            self.streamer.stop()
            self._notify("UART disconnected. The running ESP32 firmware keeps its last servo pulse widths.", "warning")
            return
        try:
            self.streamer.publish(self.state_model.packet)
            self.streamer.start(self.port.get(), int(self.baud.get()))
        except (ValueError, OSError) as error:
            self._notify(str(error), "error")
            return
        self._notify("Opening UART. The displayed joint command will stream after the ESP32 startup wait.")

    def _poll_transport(self):
        status = self.streamer.status()
        running = self.streamer.running
        self.connect_button.configure(text="Disconnect" if running else "Connect & stream")
        for widget in (self.port_combo, self.baud_combo):
            widget.configure(state="disabled" if running else "readonly")
        self.refresh_button.configure(state="disabled" if running else "normal")
        color = RED if status.state == "error" else ACCENT if status.state == "streaming" else MUTED
        self.uart_status.configure(text=f"●  {status.state.upper()}", fg=color)
        self.connection_badge.configure(text=f"●  UART {status.state.upper()}", fg=color)
        self.revision_label.configure(text=f"{self.state_model.revision} edits · {status.sent} sent")
        self._refresh_last_sent(status)
        if status.state == "streaming":
            reference_time = status.last_ack_time or status.streaming_since
            missing_ack = monotonic()-reference_time > 2.0
            ack_text = ("No recent ESP32 ACK. Check firmware, baud and UART wiring." if missing_ack else
                        f"{status.acknowledged} ACKs · last: {status.last_ack or 'waiting'} · not position feedback")
            self.ack_label.configure(text=ack_text, fg=ORANGE if missing_ack else MUTED)
            self.link_footer.configure(text="UART live · 115200 baud · 20 Hz · last valid command repeated")
        else:
            self.ack_label.configure(text=status.error or "Connect sends the shown pose after a 2.5-second startup wait.", fg=RED if status.error else MUTED)
            self.link_footer.configure(text=f"UART {status.state} · 115200 baud · 8N1")
        if self.streamer.dry_run:
            self.connection_badge.configure(text=f"●  ROS 2 DRY RUN · {status.state.upper()}")
            if status.state == "streaming":
                self.uart_status.configure(text="●  DRY RUN")
                self.ack_label.configure(text=f"Dry run · {status.acknowledged} synthetic ACKs · no serial device", fg=ORANGE)
                self.link_footer.configure(text="ROS 2 dry run · synthetic ACK/DONE · no physical output")
        for message in self.streamer.drain_events():
            if message.startswith(("UART error:", "ERR,")):
                self._stop_loop()
                self._notify(message, "error", temporary=False)
            elif message.startswith("HOLD,"):
                self._stop_loop()
                self._notify(message, "warning")
            else:
                self._log(message)
        # Process HOLD/ERR before the loop can advance at a dwell boundary.
        self._poll_loop(status)
        self.transport_job = self.after(50, self._poll_transport)

    def _refresh_last_sent(self, status):
        frame = status.last_sent.replace("\n", "\\n") if status.last_sent else "none"
        self.last_sent_label.configure(text=f"Last UART write: {frame}")

    def destroy(self):
        self.loop_player.stop()
        self.streamer.stop()
        for job in (self.transport_job, self.toast_job):
            if job is not None:
                try:
                    self.after_cancel(job)
                except tk.TclError:
                    pass
        super().destroy()

    def _copy_packet(self):
        self.clipboard_clear()
        self.clipboard_append(self.state_model.packet)
        self._notify("Copied the preview frame, including its newline terminator.")

    def _show_model(self):
        robot = self.state_model.robot
        window = tk.Toplevel(self)
        window.title("Arm Console — DH model and servo mapping")
        window.configure(bg=BG)
        window.transient(self)
        label(window, "DH model / servo calibration", 16, bold=True).pack(anchor="w", padx=24, pady=(20, 12))
        table = tk.Text(window, width=81, height=16, bg=FIELD, fg=TEXT, relief="flat", font=("Consolas", 11), padx=18, pady=14)
        table.pack(padx=24)
        lines = ["Standard DH · mm / degrees · MATLAB RRR_model.m", "",
                 "Joint    a(mm)    d(mm)    alpha     Servo conversion      DH limits"]
        for i in range(3):
            lower, upper = robot.dh_limits[i]
            lines.append(f"J{i+1}     {robot.a[i]:7.2f}  {robot.d[i]:7.2f}  {robot.alpha[i]:7.0f}°    {robot.offsets[i]:g} {robot.directions[i]:+g} × θ{i+1}       [{lower:g}, {upper:g}]°")
        lines += ["", "servo = offset + direction × theta", "theta = (servo − offset) / direction", "",
                  "Mounting: J2 = 0 horizontal, 90 upright; J3 = 90 straight.",
                  "No clipping or wrapping into 0–180: invalid solutions hold last values.",
                  "App XYZ = [DH X, DH Y, DH Z]; IK uses the inverse transform.",
                  "The ROS kinematics node supplies the active model and calibration."]
        table.insert("1.0", "\n".join(lines))
        table.configure(state="disabled")
        label(window, str(robot.config_path), 9, MUTED).pack(anchor="w", padx=24, pady=14)
        button(window, "Close", window.destroy).pack(anchor="e", padx=24, pady=(0, 20))
        window.bind("<Escape>", lambda _event: window.destroy())

    def _log(self, message):
        self.events.append(f"{datetime.now():%H:%M:%S}   {message}")
        self.events = self.events[-100:]

    def _show_activity(self):
        window = tk.Toplevel(self)
        window.title("Arm Console — Activity")
        window.geometry("780x360")
        window.configure(bg=BG)
        window.transient(self)
        label(window, "Interface activity", 16, bold=True).pack(anchor="w", padx=20, pady=16)
        text = tk.Text(window, bg=FIELD, fg=MUTED, relief="flat", wrap="word", font=("Consolas", 10), padx=14, pady=14)
        text.pack(fill="both", expand=True, padx=20, pady=(0, 20))
        text.insert("1.0", "\n".join(self.events))
        text.configure(state="disabled")
        window.bind("<Escape>", lambda _event: window.destroy())

