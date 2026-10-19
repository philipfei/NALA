"""Colours and the Qt stylesheet of the panel, after the scan screen design (Scan_screen_visualiser.html)."""

BG = '#E9EEF0'        # page background, unknown map area
PANEL = '#FFFFFF'
INK = '#10222B'       # text, walls, dark buttons
MUTE = '#5A6B73'      # secondary text
LINE = '#CBD5D9'      # borders
BLUE = '#0B5E8A'      # primary button, robot, trail
TEAL = '#2AA6A0'      # scanned floor, progress bar
AMBER = '#F2A900'     # stop banner
MAG = '#B0166B'       # contamination, hotspots
RED = '#C62828'       # STOP button
WALL = '#2B3A42'
GREEN = '#1B9E4B'     # area outline in the planner
NEW = '#E8590C'       # map change: a new object (free on the map, seen occupied)
GONE = '#74C0FC'      # map change: a removed object (occupied on the map, seen free)


def stylesheet(font_pt, button_px):
    big = button_px
    return f"""
QWidget {{ background: {BG}; color: {INK}; font-size: {font_pt}pt; }}
QLabel {{ background: transparent; }}
QFrame#side {{ background: {PANEL}; border-left: 1px solid {LINE}; }}
QFrame#sheet {{ background: {PANEL}; border: 3px solid {MAG}; border-radius: 14px; }}
QFrame#alert {{ background: {PANEL}; border: 2px solid {LINE}; border-radius: 12px; }}
QFrame#alert[hot="true"] {{ background: {MAG}; border-color: {MAG}; }}
QFrame#alert[hot="true"] QLabel {{ color: white; }}
QFrame#banner {{ background: {INK}; border-bottom: 4px solid {AMBER}; }}
QFrame#banner QLabel {{ color: white; font-weight: bold; }}
QPushButton {{ min-height: {big}px; border-radius: 12px; border: 2px solid {INK}; background: {PANEL};
              font-weight: bold; padding: 0 12px; }}
QPushButton:pressed {{ background: #DDE5E8; }}
QPushButton:disabled {{ border-color: {LINE}; color: #A9B6BB; background: #F3F6F7; }}
QPushButton[kind="primary"] {{ background: {BLUE}; border-color: {BLUE}; color: white; }}
QPushButton[kind="dark"] {{ background: {INK}; border-color: {INK}; color: white; }}
QPushButton[kind="stop"] {{ background: {RED}; border-color: {RED}; color: white; font-size: {font_pt + 6}pt; }}
QPushButton[kind="primary"]:disabled, QPushButton[kind="dark"]:disabled {{ background: #BDC7CB; border-color: #BDC7CB;
              color: #EEF2F3; }}
QPushButton[kind="toggle"] {{ border: 2px solid {LINE}; color: {MUTE}; padding: 0 1px; font-size: {font_pt - 4}pt;
              min-height: {big - 12}px; }}
QPushButton[kind="toggle"]:checked {{ border-color: {BLUE}; color: {BLUE}; background: #EAF3F8; }}
QListWidget {{ background: {PANEL}; border: 1px solid {LINE}; border-radius: 10px; }}
QListWidget::item {{ padding: 8px; border-bottom: 1px solid {LINE}; }}
QListWidget::item:selected {{ background: #EAF3F8; color: {INK}; }}
QTabWidget::pane {{ border: 0; }}
QTabBar::tab {{ min-height: {big - 12}px; padding: 0 16px; border: 2px solid {LINE}; border-bottom: 0;
               border-top-left-radius: 10px; border-top-right-radius: 10px; background: #F3F6F7; color: {MUTE}; }}
QTabBar::tab:selected {{ background: {PANEL}; color: {INK}; border-color: {BLUE}; }}
QRadioButton::indicator {{ width: 28px; height: 28px; }}
QProgressBar {{ border: 0; border-radius: 6px; background: #DDE5E8; max-height: 12px; }}
QProgressBar::chunk {{ border-radius: 6px; background: {TEAL}; }}
QPlainTextEdit {{ background: {PANEL}; border: 1px solid {LINE}; border-radius: 10px; }}
QLabel#pct {{ font-size: {font_pt * 3}pt; font-weight: bold; }}
QLabel#bigpct {{ font-size: {font_pt * 7}pt; font-weight: bold; }}
QLabel#status {{ font-weight: bold; }}
QLabel#bigstatus {{ font-size: {font_pt + 8}pt; font-weight: bold; }}
QLabel#mute {{ color: {MUTE}; }}
QLabel#chip {{ background: #E3EAED; color: {MUTE}; border-radius: 10px; padding: 2px 9px; font-size: {font_pt - 2}pt; }}
QLabel#chip[on="true"] {{ background: #D3ECEA; color: #0E5A56; }}
"""


def set_kind(button, kind):
    """Give a button its look (primary, dark, stop, toggle)."""
    button.setProperty('kind', kind)
    button.style().unpolish(button)
    button.style().polish(button)
    return button
