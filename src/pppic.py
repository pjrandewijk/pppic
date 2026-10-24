# pppic - a dpic-compatible PIC processor in pure Python (PSTricks output)
# Copyright (c) 2025, J. D. Aplevich (dpic, from which pppic is derived)
# Copyright (c) 2026, Peter Jan Randewijk (Python port)
# SPDX-License-Identifier: BSD-2-Clause
"""pppic: a pure-Python PIC processor compatible with dpic, producing PSTricks (dpic -p).

The name follows gpic and dpic; the ppp stands for Python, PSTricks and Peter.
It implements the dpic 2025.08.01 language: macros (define), for loops,
if/else, blocks, labels, positions, the object geometry of dpic.y and the
PSTricks backend of pst.c. The "sh" command is never executed.
"""

from __future__ import annotations

import argparse
import math
import random
import re
import sys
import time
from pathlib import Path

__all__ = ["PicError", "PicTimeout", "PicCompiler", "compile_pic", "main", "wfloat"]
__version__ = "1.0.0"

VERSION = "pppic (dpic 2025.08.01 compatible)"
DISTMAX = 3.40282347e38
MDISTMAX = -DISTMAX
PI = math.pi
SQRT2 = math.sqrt(2.0)

SCALED_ENV = {
    "arcrad": 0.25, "arrowht": 0.1, "arrowwid": 0.05, "boxht": 0.5, "boxrad": 0.0,
    "boxwid": 0.75, "circlerad": 0.25, "dashwid": 0.05, "ellipseht": 0.5,
    "ellipsewid": 0.75, "lineht": 0.5, "linewid": 0.5, "moveht": 0.5, "movewid": 0.5,
    "textht": 0.0, "textoffset": 2.0 / 72, "textwid": 0.0,
}
UNSCALED_ENV = {
    "arrowhead": 1.0, "fillval": 0.5, "linethick": 0.8, "maxpsht": 11.0,
    "maxpswid": 8.5, "scale": 1.0,
}
OPTION_VARS = {
    "dpicopt": 8.0, "optsafe": 0.0, "optMFpic": 1.0, "optMpost": 2.0, "optPDF": 3.0,
    "optPGF": 4.0, "optPict2e": 5.0, "optPS": 6.0, "optPSfrag": 7.0, "optPSTricks": 8.0,
    "optSVG": 9.0, "optTeX": 10.0, "opttTeX": 11.0, "optxfig": 12.0,
}
LINEAR = frozenset(("line", "arrow", "move", "spline"))
FUNC1 = ("abs", "acos", "asin", "cos", "exp", "expe", "int", "log", "loge", "sign",
         "sin", "sqrt", "tan", "floor")
FUNC2 = ("atan2", "max", "min", "pmod")
JUST_R, JUST_L, JUST_B, JUST_A = 1, 2, 4, 8
CCW_TURNS = {("up", "left"), ("down", "right"), ("right", "up"), ("left", "down")}
CW_TURNS = {("up", "right"), ("down", "left"), ("right", "down"), ("left", "up")}


class PicError(Exception):
    """An error in the PIC input."""


class PicTimeout(PicError):
    """Processing exceeded its time or loop budget."""


def rnd(x: float) -> int:
    return -int(math.floor(-x + 0.5)) if x < 0 else int(math.floor(x + 0.5))


def principal(x: float, r: float) -> float:
    while x > r:
        x -= 2 * r
    while x < -r:
        x += 2 * r
    return x


def wfloat(y: float) -> str:
    """Format a number the way dpic does (6 decimals, trailing zeros removed)."""
    sign = ""
    if y < 0:
        sign = "-"
        y = -y
    if y > 10e32:
        y = 10e32
    text = "%.6f" % (math.floor(y * 1000000 + 0.5) / 1000000.0)
    text = text.rstrip("0")
    if text.endswith("."):
        text = text[:-1]
    return sign + text


def setjust(just: int, how: str) -> int:
    if how == "rjust":
        return (just >> 2) * 4 + JUST_R
    if how == "ljust":
        return (just >> 2) * 4 + JUST_L
    if how == "below":
        return (just >> 4) * 16 + (just & 3) + JUST_B
    if how == "above":
        return (just >> 4) * 16 + (just & 3) + JUST_A
    return (just >> 4) * 16


def setangles(arcang: float, cx: float, cy: float, xs: float, ys: float, xf: float, yf: float):
    start = math.atan2(ys - cy, xs - cx)
    ra = principal(math.atan2(yf - cy, xf - cx) - start, PI)
    if ra < 0.0 and arcang > 0.0:
        ra += 2.0 * PI
    elif ra > 0.0 and arcang < 0.0:
        ra -= 2.0 * PI
    return start, ra


def affine(x: float, y: float, ox: float, oy: float, cs: tuple[float, float]):
    return ox + cs[0] * x - cs[1] * y, oy + cs[1] * x + cs[0] * y


def affang(px: float, py: float, sx: float, sy: float):
    length = math.hypot(sx - px, sy - py)
    if length == 0.0:
        return 1.0, 0.0
    return (px - sx) / length, (py - sy) / length


# ------------------------------------------------------------------ lexical tables

KEYWORDS: dict[str, tuple[str, object]] = {}
for _w, _k in (("height", "ht"), ("ht", "ht"), ("width", "wid"), ("wid", "wid"),
               ("radius", "rad"), ("rad", "rad"), ("diameter", "diam"), ("diam", "diam"),
               ("thickness", "thick"), ("thick", "thick"), ("filled", "fill"), ("fill", "fill"),
               ("undefine", "undef"), ("undef", "undef")):
    KEYWORDS[_w] = (_k, None)
for _w in ("scaled from to at with by then continue chop same of the way between and Here "
           "last print copy reset exec sh command define rand if else for do sprintf cw ccw").split():
    KEYWORDS[_w] = (_w, None)
for _w in ("st", "nd", "rd", "th"):
    KEYWORDS[_w] = ("NTH", None)
KEYWORDS.update({"top": ("CORNER", "n"), "bottom": ("CORNER", "s"),
                 "start": ("CORNER", "start"), "end": ("CORNER", "end")})
for _w in FUNC1:
    KEYWORDS[_w] = ("FUNC1", _w)
for _w in FUNC2:
    KEYWORDS[_w] = ("FUNC2", _w)
for _w, _v in (("solid", "solid"), ("dotted", "dotted"), ("dashed", "dashed"),
               ("invis", "invis"), ("invisible", "invis")):
    KEYWORDS[_w] = ("LINETYPE", _v)
for _w, _v in (("color", "color"), ("colour", "color"), ("colored", "color"),
               ("coloured", "color"), ("outline", "outline"), ("outlined", "outline"),
               ("shade", "shade"), ("shaded", "shade")):
    KEYWORDS[_w] = ("COLR", _v)
for _w, _v in (("centre", "center"), ("center", "center"), ("ljust", "ljust"),
               ("rjust", "rjust"), ("above", "above"), ("below", "below")):
    KEYWORDS[_w] = ("TEXTPOS", _v)
for _w in ("up", "down", "left", "right"):
    KEYWORDS[_w] = ("DIR", _w)
for _w in ("box", "circle", "ellipse", "arc", "line", "arrow", "move", "spline"):
    KEYWORDS[_w] = ("PRIM", _w)
for _w in list(SCALED_ENV) + list(UNSCALED_ENV):
    KEYWORDS[_w] = ("ENV", _w)

DOTWORDS: dict[str, tuple[str, object]] = {
    ".ne": ("CORNER", "ne"), ".se": ("CORNER", "se"), ".nw": ("CORNER", "nw"),
    ".sw": ("CORNER", "sw"), ".x": ("DOTX", None), ".y": ("DOTY", None),
    ".start": ("CORNER", "start"), ".end": ("CORNER", "end"),
}
for _group, _v in (((".t", ".top", ".north", ".n"), "n"),
                   ((".b", ".bot", ".bottom", ".south", ".s"), "s"),
                   ((".right", ".r", ".east", ".e"), "e"),
                   ((".left", ".l", ".west", ".w"), "w"),
                   ((".center", ".centre", ".c"), "c")):
    for _w in _group:
        DOTWORDS[_w] = ("CORNER", _v)
for _group, _v in (((".height", ".ht"), "ht"), ((".width", ".wid"), "wid"),
                   ((".radius", ".rad"), "rad"), ((".diameter", ".diam"), "diam"),
                   ((".thickness", ".thick"), "thick"), ((".length", ".len"), "len")):
    for _w in _group:
        DOTWORDS[_w] = ("PARAM", _v)

OPS2 = {
    "->": ("ARROW", "->"), "<-": ("ARROW", "<-"), "==": ("CMP", "=="), "!=": ("CMP", "!="),
    ">=": ("CMP", ">="), "<=": ("CMP", "<="), "&&": ("&&", None), "||": ("||", None),
    ":=": ("ASSIGN", ":="), "+=": ("ASSIGN", "+="), "-=": ("ASSIGN", "-="),
    "*=": ("ASSIGN", "*="), "/=": ("ASSIGN", "/="), "%=": ("ASSIGN", "%="), "[]": ("[]", None),
}
OPS1 = {">": ("CMP", ">"), "=": ("ASSIGN", "=")}
for _c in "<+-*/%^!,:()[]{}`'":
    OPS1[_c] = (_c, None)

_NL_SUPPRESS = frozenset(("else", "{", "then", ":", "NL", "do"))
_WS_RE = re.compile(r"[ \t\r\f\v]+")
_NUM_RE = re.compile(r"(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?")
_WORD_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_DOTWORD_RE = re.compile(r"\.([A-Za-z][A-Za-z0-9_]*)")
_ARGREF_RE = re.compile(r"\$(\d+)")
_SPRINTF_RE = re.compile(r"%(-?)([0-9.]*)([gfe])")


class Tok:
    __slots__ = ("kind", "value")

    def __init__(self, kind: str, value=None) -> None:
        self.kind = kind
        self.value = value

    def __repr__(self) -> str:
        return repr(self.kind) if self.value is None else f"{self.kind} {self.value!r}"


class Frame:
    __slots__ = ("text", "pos", "kind", "args")

    def __init__(self, text: str, kind: str, args=None) -> None:
        self.text = text
        self.pos = 0
        self.kind = kind
        self.args = args


class Lexer:
    """dpic lexical analysis, including macro expansion and raw block reading."""

    def __init__(self, compiler: "PicCompiler", text: str) -> None:
        self.compiler = compiler
        self.frames = [Frame(text, "src")]
        self.last = "NL"

    def emit(self, kind: str, value=None) -> Tok:
        self.last = kind
        return Tok(kind, value)

    def push(self, text: str, kind: str, args=None) -> None:
        if len(self.frames) > 20000:
            raise PicError("macro or input nesting too deep (infinite recursion?)")
        self.frames.append(Frame(text, kind, args))

    def context(self) -> str:
        for frame in reversed(self.frames):
            if frame.pos < len(frame.text):
                return frame.text[frame.pos : frame.pos + 60].replace("\n", "\\n")
        return "<end of input>"

    def next(self) -> Tok:
        frames = self.frames
        while True:
            if not frames:
                return self.emit("EOF")
            f = frames[-1]
            text = f.text
            pos = f.pos
            if pos >= len(text):
                frames.pop()
                if f.kind == "body":
                    return self.emit("END")
                continue
            c = text[pos]
            if c in " \t\r\f\v":
                f.pos = _WS_RE.match(text, pos).end()
                continue
            if c == "\n" or c == ";":
                f.pos = pos + 1
                if self.last in _NL_SUPPRESS:
                    continue
                return self.emit("NL")
            if c == "\\":
                nxt = text[pos + 1 : pos + 2]
                if nxt == "\n" or nxt == "":
                    f.pos = pos + 2
                    continue
                if nxt == "#":
                    j = text.find("\n", pos)
                    f.pos = len(text) if j < 0 else j + 1
                    continue
                j = text.find("\n", pos)
                end = len(text) if j < 0 else j
                f.pos = end
                return self.emit("LATEX", text[pos:end])
            if c == "#":
                j = text.find("\n", pos)
                if j >= 0:
                    f.pos = j
                    continue
                f.pos = len(text)
                if self.last in _NL_SUPPRESS:
                    continue
                return self.emit("NL")
            if c == '"':
                return self.emit("STRING", self._read_string(f))
            if ("0" <= c <= "9") or (c == "." and "0" <= text[pos + 1 : pos + 2] <= "9" and text[pos + 1 : pos + 2] != ""):
                m = _NUM_RE.match(text, pos)
                f.pos = m.end()
                if text[f.pos : f.pos + 1] == "i":
                    f.pos += 1
                return self.emit("NUM", float(m.group()))
            if ("a" <= c <= "z") or ("A" <= c <= "Z") or c == "_":
                m = _WORD_RE.match(text, pos)
                while text.startswith("$", m.end()) and text[m.end() + 1 : m.end() + 2].isdigit():
                    text = self._splice_arg(f, m.end())
                    m = _WORD_RE.match(text, pos)
                word = m.group()
                f.pos = m.end()
                kw = KEYWORDS.get(word)
                if kw is not None:
                    return self.emit(kw[0], kw[1])
                if word in self.compiler.macros and self.last not in ("define", "undef"):
                    self._expand(word, f)
                    continue
                return self.emit("LABEL" if "A" <= word[0] <= "Z" else "NAME", word)
            if c == ".":
                m = _DOTWORD_RE.match(text, pos)
                if m:
                    word = "." + m.group(1)
                    hit = DOTWORDS.get(word)
                    if hit is not None:
                        f.pos = m.end()
                        return self.emit(*hit)
                    if not ("A" <= m.group(1)[0] <= "Z"):
                        for length in range(len(word) - 1, 1, -1):
                            hit = DOTWORDS.get(word[:length])
                            if hit is not None:
                                f.pos = pos + length
                                return self.emit(*hit)
                f.pos = pos + 1
                return self.emit(".")
            if c == "$":
                if text[pos + 1 : pos + 2] == "+":
                    f.pos = pos + 2
                    return self.emit("NUM", float(len(self.current_args() or ())))
                if text[pos + 1 : pos + 2].isdigit():
                    self._splice_arg(f, pos)
                    continue
                f.pos = pos + 1
                continue
            if c == "&" and text[pos + 1 : pos + 2] != "&":
                f.pos = pos + 2
                continue
            if text.startswith("<->", pos):
                f.pos = pos + 3
                return self.emit("ARROW", "<->")
            hit = OPS2.get(text[pos : pos + 2])
            if hit is not None:
                f.pos = pos + 2
                return self.emit(*hit)
            hit = OPS1.get(c)
            if hit is not None:
                f.pos = pos + 1
                return self.emit(*hit)
            f.pos = pos + 1
            self.compiler.error(f"invalid character {c!r}")

    def current_args(self):
        """Arguments of the innermost macro whose body is being read (dpic `args`), or None."""
        for frame in reversed(self.frames):
            if frame.args is not None:
                return frame.args
        return None

    def _arg_text(self, n: int, args=None) -> str:
        args = self.current_args() if args is None else args
        return args[n - 1] if args and 1 <= n <= len(args) else ""

    def _splice_arg(self, f: Frame, pos: int) -> str:
        """Replace $n at pos in the frame text by the argument text (dpic insertarg)."""
        m = _ARGREF_RE.match(f.text, pos)
        f.text = f.text[:pos] + self._arg_text(int(m.group(1))) + f.text[m.end():]
        f.pos = pos if f.pos > pos else f.pos
        return f.text

    def _read_string(self, f: Frame) -> str:
        text = f.text
        i = f.pos + 1
        out = []
        args = self.current_args()
        while i < len(text):
            c = text[i]
            if c == "\\" and text[i + 1 : i + 2] == '"':
                out.append('"')
                i += 2
                continue
            if c == '"':
                f.pos = i + 1
                return "".join(out)
            if c == "$" and args is not None and text[i + 1 : i + 2].isdigit():
                m = _ARGREF_RE.match(text, i)
                out.append(self._arg_text(int(m.group(1)), args))
                i = m.end()
                continue
            out.append(c)
            i += 1
        f.pos = len(text)
        raise PicError("end of input inside a string")

    # ---------------------------------------------------------- macros

    def _expand(self, name: str, f: Frame) -> None:
        body = self.compiler.macros[name]
        text = f.text
        i = f.pos
        args: list[str] = []
        if i < len(text) and text[i] == "(":
            args, i = self._macro_args(text, i + 1, self.current_args())
        f.pos = i
        if f.pos >= len(text) and f.kind in ("macro", "exec"):
            self.frames.pop()
        self.push(body, "macro", args)

    def _macro_args(self, text: str, i: int, caller_args: list):
        """Read macro arguments like dpic copyargbody: $n refers to the caller's arguments."""
        args: list[str] = []
        n = len(text)
        while True:
            while i < n and text[i] in " \t\r\n":
                i += 1
            out = []
            start = i
            level = 0
            bracket = 0
            instring = False
            prev = ""
            while True:
                if i >= n:
                    raise PicError("end of input in macro arguments")
                c = text[i]
                if prev != "\\":
                    if instring and c == '"':
                        instring = False
                    elif c == '"':
                        instring = True
                if not instring:
                    if c == "[":
                        bracket += 1
                    elif c == "]":
                        bracket -= 1
                    elif c == "(":
                        level += 1
                    elif c == ")":
                        level -= 1
                if not instring and bracket == 0 and (level < 0 or (level == 0 and c == ",")):
                    out.append(text[start:i])
                    args.append("".join(out))
                    i += 1
                    break
                if c == "$" and text[i + 1 : i + 2].isdigit():
                    m = _ARGREF_RE.match(text, i)
                    out.append(text[start:i])
                    out.append(self._arg_text(int(m.group(1)), caller_args))
                    i = start = m.end()
                    prev = ""
                    continue
                prev = c
                i += 1
            if level < 0:
                return args, i

    def _raw_frame(self) -> Frame:
        while self.frames:
            f = self.frames[-1]
            if f.pos < len(f.text) or f.kind == "body":
                return f
            self.frames.pop()
        raise PicError("end of input while reading a { } block")

    def read_braced(self, keep: bool = True) -> str:
        """Read raw text after an opening brace up to the matching closing brace."""
        f = self._raw_frame()
        text = f.text
        i = f.pos
        n = len(text)
        level = 1
        instring = False
        prev = ""
        out = []
        start = i
        while i < n:
            c = text[i]
            if instring:
                if c == '"' and prev != "\\":
                    instring = False
            elif c == "#":
                j = text.find("\n", i)
                j = n if j < 0 else j
                if keep:
                    out.append(text[start:i])
                start = j
                i = j
                prev = ""
                continue
            elif c == "{":
                level += 1
            elif c == "}":
                level -= 1
                if level == 0:
                    if keep:
                        out.append(text[start:i])
                    f.pos = i + 1
                    self.last = "}"
                    return "".join(out)
            elif c == '"':
                instring = True
            prev = c
            i += 1
        f.pos = n
        raise PicError("end of input while reading a { } block")

    def read_macro_body(self) -> str:
        f = self._raw_frame()
        text = f.text
        i = f.pos
        n = len(text)
        while i < n:
            if text[i] in " \t\r\n":
                i += 1
            elif text[i] == "\\" and text[i + 1 : i + 2] in ("\n", "\r"):
                i += 2
            else:
                break
        if i >= n:
            raise PicError("missing macro body")
        delim = text[i]
        f.pos = i + 1
        if delim == "{":
            return self.read_braced()
        j = i + 1
        instring = False
        while j < n:
            c = text[j]
            if instring:
                if c == '"' and text[j - 1] != "\\":
                    instring = False
            elif c == '"':
                instring = True
            elif c == delim:
                f.pos = j + 1
                self.last = "}"
                return text[i + 1 : j]
            j += 1
        raise PicError("end of input in macro body")


# ------------------------------------------------------------------ objects


class Seg:
    __slots__ = ("x0", "y0", "x1", "y1", "texts", "atype", "ahnum")

    def __init__(self, x0: float, y0: float, x1: float, y1: float) -> None:
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1
        self.texts: list[list] = []
        self.atype = ""
        self.ahnum = 1


class Prim:
    def __init__(self, kind: str, parent, x: float, y: float, direction) -> None:
        self.kind = kind
        self.name = None
        self.parent = parent
        self.x = x
        self.y = y
        self.direction = direction
        self.texts: list[list] = []
        self.outline = None
        self.shaded = None
        self.lparam = MDISTMAX
        self.lthick = MDISTMAX
        self.spec = "solid"
        self.fill = -1.0
        self.w = 0.0
        self.h = 0.0
        self.brad = 0.0
        self.r = 0.0
        self.ah = 0.0
        self.aw = 0.0
        self.atype = ""
        self.ahnum = 1
        self.tension = MDISTMAX
        self.arad = 0.0
        self.sang = 0.0
        self.aang = 0.0
        self.segs = None
        self.children = None
        self.here = None
        self.vars = None
        self.env = None
        self.text = None


class ObjState:
    def __init__(self, prim: Prim) -> None:
        self.prim = prim
        self.flags: set[str] = set()
        self.at = (0.0, 0.0)
        self.corner = "c"
        self.pair = (0.0, 0.0)
        self.internal = None
        self.startchop = 0.0
        self.endchop = 0.0
        self.toklen = None
        self.continued = False


# ------------------------------------------------------------------ compiler


class PicCompiler:
    """Interpret PIC source and render PSTricks output exactly as dpic -p would."""

    def __init__(self, *, base_dir=None, include_dirs=None, deadline: float | None = None,
                 loop_limit: int = 5_000_000) -> None:
        self.base_dir = Path(base_dir) if base_dir is not None else Path.cwd()
        self.include_dirs = [Path(d) for d in (include_dirs or [])]
        self.macros: dict[str, str] = {}
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.messages: list[str] = []
        self.deadline = deadline
        self.loop_budget = loop_limit
        self.rng = random.Random(0)
        self.lexer: Lexer | None = None
        self.tokbuf: list[Tok] = []
        self.out: list[str] = []
        self.scale = 1.0
        self.fsc = 1.0
        self.lastfillval = MDISTMAX
        self.gb: Prim | None = None
        self.envblock: Prim | None = None
        self._reported: set[str] = set()

    # ---------------------------------------------------------- diagnostics

    def error(self, message: str) -> None:
        context = self.lexer.context() if self.lexer is not None else ""
        self.errors.append(f"{message} [near: {context}]")

    def warn(self, message: str) -> None:
        if message not in self._reported:
            self._reported.add(message)
            self.warnings.append(message)

    # ---------------------------------------------------------- driver

    def compile(self, source: str) -> str:
        source = source.replace("\r\n", "\n").replace("\r", "\n")
        lines = source.split("\n")
        if lines and lines[-1] == "":
            lines.pop()
        out: list[str] = []
        i = 0
        while i < len(lines):
            line = lines[i]
            if line.startswith(".PS") or line.startswith(".PF"):
                args = [float(v) for v in re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?",
                                                     line[3:].split("#", 1)[0])][:2]
                body: list[str] = []
                i += 1
                continued = False
                while i < len(lines):
                    current = lines[i]
                    if current.startswith(".") and not continued:
                        if current.startswith(".PE"):
                            break
                        i += 1
                        continue
                    body.append(current)
                    continued = current.endswith("\\")
                    i += 1
                out.append(self.run_picture("\n".join(body) + "\n", args))
                i += 1
                continue
            out.append(line + "\n")
            i += 1
        return "".join(out)

    def run_picture(self, body: str, args: list[float]) -> str:
        gb = Prim("block", None, 0.0, 0.0, "right")
        gb.here = [0.0, 0.0]
        gb.children = []
        gb.vars = dict(OPTION_VARS)
        gb.env = {**SCALED_ENV, **UNSCALED_ENV}
        self.gb = gb
        self.envblock = gb
        self.lastfillval = MDISTMAX
        self.lexer = Lexer(self, body)
        self.tokbuf = []
        self.parse_elementlist({"EOF"})
        xv = args[0] if args else 0.0
        yv = args[1] if len(args) > 1 else 0.0
        return self.draw_picture(xv, yv)

    # ---------------------------------------------------------- tokens

    def peek(self, k: int = 0) -> Tok:
        buf = self.tokbuf
        while len(buf) <= k:
            buf.append(self.lexer.next())
        return buf[k]

    def next(self) -> Tok:
        if self.tokbuf:
            return self.tokbuf.pop(0)
        return self.lexer.next()

    def expect(self, kind: str) -> Tok:
        t = self.next()
        if t.kind != kind:
            raise PicError(f"expected {kind!r} but found {t}")
        return t

    def _no_lookahead(self) -> None:
        if self.tokbuf:
            raise PicError(f"unexpected {self.tokbuf[0]} before a {{ }} block")

    # ---------------------------------------------------------- environment

    def findenv(self, p: Prim) -> Prim:
        q = p
        while q is not None:
            if q.kind == "block" and q.env is not None:
                return q
            q = q.parent
        return self.gb

    def env(self) -> dict:
        return self.findenv(self.envblock).env

    def venv(self, p: Prim, name: str) -> float:
        return self.findenv(p).env[name]

    def qenv(self, p: Prim, name: str, local: float) -> float:
        noval = -1.0 if name == "fillval" else MDISTMAX if name in ("linethick", "dashwid") else 0.0
        if local == noval:
            return self.venv(p, name)
        if name == "linethick":
            return abs(local)
        return local

    def inheritenv(self, blk: Prim) -> None:
        source = self.findenv(blk.parent) if blk.parent is not None else None
        if source is None or source.env is None:
            blk.env = {**SCALED_ENV, **UNSCALED_ENV}
        else:
            blk.env = dict(source.env)

    def resetscale(self, value: float, op: str, blk: Prim) -> None:
        env = blk.env
        env.update(SCALED_ENV)
        old = env["scale"]
        env["scale"] = self.eqop(old, op, value)
        new = env["scale"]
        if new == 0.0:
            env["scale"] = old
            new = 1.0
            self.error("scale must not be zero")
        elif old == 0.0:
            self.error("division by zero")
        else:
            new /= old
        for name in SCALED_ENV:
            env[name] *= new

    def resetenv(self, blk: Prim, name: str | None) -> None:
        if blk.env is None:
            self.inheritenv(blk)
        if name is None:
            blk.env.update(SCALED_ENV)
            blk.env.update(UNSCALED_ENV)
        elif name in SCALED_ENV:
            blk.env[name] = SCALED_ENV[name]
        else:
            blk.env[name] = UNSCALED_ENV[name]

    def eqop(self, x: float, op: str, y: float) -> float:
        if op in ("=", ":="):
            return y
        if op == "+=":
            return x + y
        if op == "-=":
            return x - y
        if op == "*=":
            return x * y
        if op == "/=":
            if y == 0:
                self.error("division by zero")
                return x
            return x / y
        i, j = rnd(x), rnd(y)
        if j == 0:
            self.error("division by zero")
            return 0.0
        return float(i - _c_trunc_div(i, j) * j)

    # ---------------------------------------------------------- statements

    def parse_elementlist(self, terms: set[str]) -> None:
        while True:
            if self.deadline is not None and time.monotonic() > self.deadline:
                raise PicTimeout("time limit exceeded")
            t = self.peek()
            if t.kind in terms:
                return
            if t.kind == "NL":
                self.next()
                continue
            if t.kind in ("EOF", "END"):
                raise PicError("unexpected end of input (missing } or ]?)")
            try:
                self.parse_element()
                t = self.peek()
                if t.kind == "NL":
                    self.next()
                elif t.kind not in terms and t.kind not in ("EOF", "END"):
                    raise PicError(f"syntax error near {t}")
            except PicTimeout:
                raise
            except PicError as err:
                self.error(str(err))
                self.recover(terms)

    def recover(self, terms: set[str]) -> None:
        while True:
            t = self.peek()
            if t.kind == "NL":
                self.next()
                return
            if t.kind in terms or t.kind in ("EOF", "END"):
                return
            self.next()

    def parse_element(self) -> None:
        t = self.peek()
        k = t.kind
        if k == "LABEL":
            self.next()
            name = t.value + self.parse_suffix()
            self.expect(":")
            if self.is_object_start(self.peek()):
                st = self.parse_object()
                self.finish_object(st, name)
            else:
                x, y = self.parse_position()
                self.define_place(name, x, y)
        elif k in ("NAME", "ENV"):
            self.parse_assignment()
            while self.peek().kind == ",":
                self.next()
                self.parse_assignment()
        elif k == "DIR":
            self.next()
            self.envblock.direction = t.value
        elif k == "LATEX":
            self.next()
            self.queue(self.make_latex(text=t.value))
        elif k == "{":
            self.parse_group()
        elif k == "if":
            self.parse_if()
        elif k == "for":
            self.parse_for()
        elif k == "command":
            self.next()
            self.queue(self.make_latex(text=self.parse_stringexpr()))
        elif k == "exec":
            self.next()
            self.lexer.push("\n" + self.parse_stringexpr() + "\n", "exec")
        elif k == "print":
            self.parse_print()
        elif k == "reset":
            self.next()
            if self.peek().kind == "ENV":
                names = [self.next().value]
                while self.peek().kind == ",":
                    self.next()
                    names.append(self.expect("ENV").value)
                for name in names:
                    self.resetenv(self.envblock, name)
            else:
                self.resetenv(self.envblock, None)
        elif k == "sh":
            self.next()
            self.parse_stringexpr()
            self.warn("sh command ignored: pppic does not run shell commands")
        elif k == "copy":
            self.next()
            self.copy_file(self.parse_stringexpr())
        elif k == "define":
            self.next()
            self._no_lookahead()
            name = self.lexer.next()
            if name.kind not in ("NAME", "LABEL"):
                raise PicError(f"invalid macro name {name}")
            self.macros[name.value] = self.lexer.read_macro_body()
        elif k == "undef":
            self.next()
            self._no_lookahead()
            name = self.lexer.next()
            self.macros.pop(str(name.value), None)
        elif self.is_object_start(t):
            st = self.parse_object()
            self.finish_object(st, None)
        else:
            raise PicError(f"syntax error near {t}")

    def parse_group(self) -> None:
        self.next()
        blk = self.envblock
        hx, hy = blk.here
        direction = blk.direction
        try:
            self.parse_elementlist({"}"})
            self.expect("}")
        finally:
            blk.here = [hx, hy]
            blk.direction = direction

    def parse_if(self) -> None:
        self.next()
        cond = self.truth(self.parse_logic())
        self.expect("then")
        self.expect("{")
        if cond:
            self.parse_elementlist({"}"})
            self.expect("}")
            if self.peek().kind == "else":
                self.next()
                self.expect("{")
                self._no_lookahead()
                self.lexer.read_braced(keep=False)
        else:
            self._no_lookahead()
            self.lexer.read_braced(keep=False)
            if self.peek().kind == "else":
                self.next()
                self.expect("{")
                self.parse_elementlist({"}"})
                self.expect("}")

    def parse_for(self) -> None:
        self.next()
        x0, holder, key = self.parse_assignment()
        while self.peek().kind == ",":
            self.next()
            self.parse_assignment()
        self.expect("to")
        final = self.scalar(self.parse_expression())
        inc = 1.0
        mult = False
        if self.peek().kind == "by":
            self.next()
            if self.peek().kind == "*":
                self.next()
                mult = True
            inc = self.scalar(self.parse_expression())
        self.expect("do")
        self.expect("{")
        self._no_lookahead()
        body = self.lexer.read_braced()
        mode = 0
        if x0 == final:
            mode = -1
            inc = 1.0
        elif mult:
            t = final * x0
            if x0 == 0 or t < 0 or (t == 0 and abs(inc * x0) != 0) or (inc == 0 and t != 0) or \
                    (abs(inc) == 1 and abs(x0) != abs(final)):
                self.error("invalid multiplicative for loop")
            else:
                mode = 1
        elif inc == 0:
            self.error("zero for-loop increment")
        elif (final - x0) * inc > 0:
            mode = -1
        if mode == 0:
            return
        while True:
            self.lexer.push(body + "\n", "body")
            self.parse_elementlist({"END"})
            self.expect("END")
            x = holder[key]
            if mode < 0:
                x += inc
                stop = (inc > 0 and x > final) or (inc < 0 and x < final)
            elif x == 0:
                stop = True
            else:
                x *= inc
                stop = (abs(inc) >= 1 and abs(x) > abs(final)) or (abs(inc) < 1 and abs(x) < abs(final))
            if key == "scale" and holder is self.findenv(self.envblock).env:
                self.resetscale(x, "=", self.findenv(self.envblock))
            else:
                holder[key] = x
            if stop:
                return
            self.loop_budget -= 1
            if self.loop_budget < 0:
                raise PicTimeout("for-loop iteration limit exceeded")

    def parse_print(self) -> None:
        self.next()
        if self.peek().kind in ("STRING", "sprintf"):
            message = self.parse_stringexpr()
        else:
            value = self.parse_position(allow_scalar=True)
            if isinstance(value, tuple):
                message = f"({wfloat(value[0])},{wfloat(value[1])})"
            else:
                message = wfloat(value)
        t = self.peek()
        if t.kind == "CMP" and t.value == ">":
            self.next()
            if self.peek().kind == "CMP":
                self.next()
            self.parse_stringexpr()
            self.warn("print redirection to a file ignored")
        self.messages.append(message)

    def copy_file(self, name: str) -> None:
        path = Path(name)
        candidates = [path] if path.is_absolute() else \
            [self.base_dir / path] + [d / path for d in self.include_dirs] + [path]
        for candidate in candidates:
            if candidate.is_file():
                data = candidate.read_bytes()
                try:
                    text = data.decode("utf-8")
                except UnicodeDecodeError:
                    text = data.decode("cp1252")
                text = text.replace("\r\n", "\n").replace("\r", "\n")
                text = "\n".join(line for line in text.split("\n") if not line.startswith("."))
                self.lexer.push(text + "\n", "src")
                return
        self.error(f"cannot read copy file {name!r}")

    # ---------------------------------------------------------- assignments

    def parse_assignment(self):
        t = self.next()
        if t.kind == "ENV":
            op = self.expect("ASSIGN").value
            value = self.parse_assign_rhs()
            return self.assign_env(t.value, op, value)
        if t.kind == "NAME":
            name = t.value + self.parse_suffix()
            op = self.expect("ASSIGN").value
            value = self.parse_assign_rhs()
            return self.assign_var(name, op, value)
        raise PicError(f"syntax error near {t}")

    def parse_assign_rhs(self) -> float:
        t = self.peek()
        if t.kind in ("NAME", "ENV") and self.peek(1).kind == "ASSIGN":
            return self.parse_assignment()[0]
        return self.scalar(self.parse_expression())

    def assign_var(self, name: str, op: str, value: float):
        blk = self.envblock
        holder = blk.vars if name in blk.vars else None
        if holder is None and op != "=":
            q = blk.parent
            while q is not None:
                if name in q.vars:
                    holder = q.vars
                    break
                q = q.parent
        if holder is None:
            holder = blk.vars
            holder[name] = 0.0
        holder[name] = value if op == "=" else self.eqop(holder[name], op, value)
        return holder[name], holder, name

    def assign_env(self, name: str, op: str, value: float):
        blk = self.envblock
        if blk.env is None:
            self.inheritenv(blk)
        if name == "scale":
            self.resetscale(value, op, blk)
        else:
            blk.env[name] = self.eqop(blk.env[name], op, value)
        result = blk.env[name]
        if name == "linethick":
            self.queue(self.make_latex(lthick=result))
        return result, blk.env, name

    def lookup_var(self, name: str) -> float:
        q = self.envblock
        while q is not None:
            if name in q.vars:
                return q.vars[name]
            q = q.parent
        self.error(f"variable {name} not found")
        return 0.0

    def parse_suffix(self) -> str:
        if self.peek().kind != "[":
            return ""
        self.next()
        value = self.parse_position(allow_scalar=True)
        self.expect("]")
        if isinstance(value, tuple):
            return f"[{rnd(value[0])},{rnd(value[1])}]"
        return f"[{rnd(value)}]"

    # ---------------------------------------------------------- places and labels

    @staticmethod
    def findplace(children, name: str):
        for child in children or ():
            if child.name == name:
                return child
        return None

    def find_label(self, name: str):
        q = self.envblock
        while q is not None:
            p = self.findplace(q.children, name)
            if p is not None:
                return p
            q = q.parent
        return None

    @staticmethod
    def nthprimobj(children, nth: int, typ: str):
        children = children or ()
        if nth == 0:
            found = None
            for child in children:
                if child.kind == typ:
                    found = child
            return found
        if nth < 0:
            nth += sum(1 for child in children if child.kind == typ) + 1
        count = 0
        for child in children:
            if child.kind == typ:
                count += 1
                if count == nth:
                    return child
        return None

    def define_place(self, name: str, x: float, y: float) -> None:
        blk = self.envblock
        p = self.findplace(blk.children, name)
        if p is not None and p.kind == "label":
            p.x, p.y = x, y
            return
        label = Prim("label", blk, x, y, blk.direction)
        label.spec = "invis"
        label.name = name
        if p is not None:
            p.name = None
        self.queue(label)

    def queue(self, p: Prim) -> None:
        self.envblock.children.append(p)

    def make_latex(self, text: str | None = None, lthick: float = MDISTMAX) -> Prim:
        blk = self.envblock
        p = Prim("latex", blk, blk.here[0], blk.here[1], blk.direction)
        p.spec = "invis"
        p.text = text
        p.lthick = lthick
        return p

    # ---------------------------------------------------------- objects

    @staticmethod
    def is_object_start(t: Tok) -> bool:
        return t.kind in ("PRIM", "STRING", "sprintf", "[", "[]", "continue")

    def new_prim(self, kind: str) -> Prim:
        blk = self.envblock
        p = Prim(kind, blk, blk.here[0], blk.here[1], blk.direction)
        if kind not in ("string", "spline", "arc", "arrow", "line", "ellipse", "circle", "box"):
            p.spec = "invis"
        return p

    def parse_object(self) -> ObjState:
        t = self.peek()
        k = t.kind
        blk = self.envblock
        if k == "PRIM":
            self.next()
            opt = self.parse_optexp() if t.value in LINEAR else None
            p = self.new_prim(t.value)
            self.init_primitive(p, opt)
            st = ObjState(p)
        elif k in ("STRING", "sprintf"):
            text = self.parse_stringexpr()
            p = self.new_prim("string")
            env = self.env()
            p.w = env["textwid"]
            p.h = env["textht"]
            p.texts.append([text, 0])
            st = ObjState(p)
        elif k == "[":
            self.next()
            p = self.new_prim("block")
            p.here = [0.0, 0.0]
            p.children = []
            p.vars = {}
            self.envblock = p
            try:
                self.parse_elementlist({"]"})
                self.expect("]")
                if p.env is not None:
                    outer = self.findenv(blk).env
                    if p.env["linethick"] != outer["linethick"]:
                        p.children.append(self.make_latex(lthick=outer["linethick"]))
            finally:
                self.envblock = blk
            self.close_block(p)
            st = ObjState(p)
        elif k == "[]":
            self.next()
            p = self.new_prim("block")
            p.here = [0.0, 0.0]
            p.children = []
            p.vars = {}
            st = ObjState(p)
        elif k == "continue":
            self.next()
            target = None
            for child in blk.children:
                if child.kind not in ("latex", "string", "block", "label"):
                    target = child
            if target is None or target.segs is None:
                raise PicError("there is no line to continue")
            st = ObjState(target)
            st.continued = True
            self.attr_then(st)
        else:
            raise PicError(f"an object was expected, found {t}")
        self.parse_attributes(st)
        return st

    def init_primitive(self, p: Prim, opt) -> None:
        env = self.env()
        kind = p.kind
        if kind != "move":
            p.lthick = env["linethick"] if kind == "arc" else -env["linethick"]
        d = p.direction
        if kind == "box":
            p.h, p.w, p.brad = env["boxht"], env["boxwid"], env["boxrad"]
            self.offset_closed(p, p.w / 2, p.h / 2)
        elif kind == "circle":
            p.r = env["circlerad"]
            self.offset_closed(p, p.r, p.r)
        elif kind == "ellipse":
            p.h, p.w = env["ellipseht"], env["ellipsewid"]
            self.offset_closed(p, p.w / 2, p.h / 2)
        elif kind == "arc":
            p.arad = env["arcrad"]
            if d == "up":
                p.sang = 0.0
                p.x -= p.arad
            elif d == "down":
                p.sang = PI
                p.x += p.arad
            elif d == "left":
                p.sang = 0.5 * PI
                p.y -= p.arad
            else:
                p.sang = -0.5 * PI
                p.y += p.arad
            p.ah, p.aw = env["arrowht"], env["arrowwid"]
            p.ahnum = rnd(env["arrowhead"])
            p.aang = 0.5 * PI
        else:
            if kind == "spline" and opt is not None:
                p.tension = opt
                opt = None
            if opt is not None:
                length = opt
            elif kind == "move":
                length = env["moveht"] if d in ("up", "down") else env["movewid"]
            else:
                length = env["lineht"] if d in ("up", "down") else env["linewid"]
            x1, y1 = self.step(p.x, p.y, d, length)
            seg = Seg(p.x, p.y, x1, y1)
            seg.atype = "->" if kind == "arrow" else ""
            seg.ahnum = rnd(env["arrowhead"])
            p.segs = [seg]
            p.ah, p.aw = env["arrowht"], env["arrowwid"]

    @staticmethod
    def step(x: float, y: float, d, length: float):
        if d == "up":
            return x, y + length
        if d == "down":
            return x, y - length
        if d == "left":
            return x - length, y
        return x + length, y

    @staticmethod
    def offset_closed(p: Prim, hw: float, hh: float) -> None:
        if p.direction == "up":
            p.y += hh
        elif p.direction == "down":
            p.y -= hh
        elif p.direction == "left":
            p.x -= hw
        else:
            p.x += hw

    def close_block(self, p: Prim) -> None:
        n, s, e, w = self.getnesw(p.children)
        p.w = e - w
        p.h = n - s
        p.x = (e + w) * 0.5
        p.y = (n + s) * 0.5
        outer = p.parent
        dx = outer.here[0] - p.x
        dy = outer.here[1] - p.y
        d = outer.direction
        if d == "right":
            dx += p.w * 0.5
        elif d == "left":
            dx -= p.w * 0.5
        elif d == "up":
            dy += p.h * 0.5
        elif d == "down":
            dy -= p.h * 0.5
        p.direction = d
        self.shift(p, dx, dy)

    def shift(self, p: Prim, dx: float, dy: float) -> None:
        if dx == 0 and dy == 0:
            return
        if p.segs is not None:
            for seg in p.segs:
                seg.x0 += dx
                seg.y0 += dy
                seg.x1 += dx
                seg.y1 += dy
        else:
            p.x += dx
            p.y += dy
        if p.children:
            for child in p.children:
                self.shift(child, dx, dy)

    def scaleobj(self, p: Prim, s: float) -> None:
        if p.segs is not None:
            for seg in p.segs:
                seg.x0 *= s
                seg.y0 *= s
                seg.x1 *= s
                seg.y1 *= s
        else:
            p.x *= s
            p.y *= s
        if p.kind == "box":
            p.h *= s
            p.w *= s
            p.brad *= s
        elif p.kind in ("block", "ellipse"):
            p.h *= s
            p.w *= s
        elif p.kind == "circle":
            p.r *= s
        elif p.kind == "arc":
            p.arad *= s
        if p.children:
            for child in p.children:
                self.scaleobj(child, s)

    def arcstart(self, p: Prim):
        return p.x + p.arad * math.cos(p.sang), p.y + p.arad * math.sin(p.sang)

    def arcend(self, p: Prim):
        return (p.x + p.arad * math.cos(p.sang + p.aang),
                p.y + p.arad * math.sin(p.sang + p.aang))

    def finish_object(self, st: ObjState, name: str | None) -> None:
        p = st.prim
        blk = self.envblock
        if name is not None:
            old = self.findplace(blk.children, name)
            if old is not None:
                old.name = None
            p.name = name
        if p.kind == "arc":
            self.arcenddir(p)
        if "at" in st.flags:
            self.deferred_shift(st)
        if p.kind in LINEAR:
            self.apply_chop(st)
        blk.here = list(self.exit_point(p))
        if not st.continued:
            self.queue(p)

    def arcenddir(self, p: Prim) -> None:
        d = p.direction
        if d is None:
            return
        if p.aang > 0.0:
            turn = {"up": "left", "down": "right", "left": "down", "right": "up"}
        else:
            turn = {"up": "right", "down": "left", "left": "up", "right": "down"}
        self.envblock.direction = turn[d]

    def deferred_shift(self, st: ObjState) -> None:
        p = st.prim
        which = st.corner
        if which == "pair":
            n, s, e, w = self.getnesw([p])
            dx, dy = w + st.pair[0], s + st.pair[1]
        elif st.internal is not None:
            dx, dy = self.corner(st.internal, which)
        else:
            dx, dy = self.corner(p, which)
        tx, ty = st.at
        if p.kind != "arc":
            self.shift(p, tx - dx, ty - dy)
            return
        x1, z1 = self.arcstart(p)
        if "to" in st.flags:
            if which not in (None, "c"):
                self.error("only the centre of an arc can be placed with 'at'")
            r, s = self.arcend(p)
            p.x, p.y = tx, ty
            p.arad = math.hypot(r - p.x, s - p.y)
            p.sang, p.aang = setangles(p.aang, p.x, p.y, x1, z1, r, s)
        elif "from" in st.flags:
            if which not in (None, "c"):
                self.error("only the centre of an arc can be placed with 'at'")
            p.x, p.y = tx, ty
            t = math.atan2(z1 - p.y, x1 - p.x)
            r = p.x + p.arad * math.cos(t + p.aang)
            s = p.y + p.arad * math.sin(t + p.aang)
            p.arad = math.hypot(x1 - p.x, z1 - p.y)
            p.sang, p.aang = setangles(p.aang, p.x, p.y, x1, z1, r, s)
        else:
            self.shift(p, tx - dx, ty - dy)

    def apply_chop(self, st: ObjState) -> None:
        segs = st.prim.segs
        if st.startchop != 0.0:
            seg = segs[0]
            dx, dy = seg.x1 - seg.x0, seg.y1 - seg.y0
            length = math.hypot(dx, dy)
            if length != 0.0:
                t = st.startchop / length
                seg.x0 += t * dx
                seg.y0 += t * dy
        if st.endchop != 0.0:
            seg = segs[-1]
            dx, dy = seg.x1 - seg.x0, seg.y1 - seg.y0
            length = math.hypot(dx, dy)
            if length != 0.0:
                t = st.endchop / length
                seg.x1 -= t * dx
                seg.y1 -= t * dy

    def exit_point(self, p: Prim):
        k = p.kind
        if k in LINEAR:
            seg = p.segs[-1]
            return seg.x1, seg.y1
        if k == "arc":
            return self.arcend(p)
        if k in ("box", "string", "block", "ellipse"):
            hw, hh = p.w * 0.5, p.h * 0.5
        elif k == "circle":
            hw = hh = p.r
        else:
            return p.x, p.y
        d = p.direction
        if d == "up":
            return p.x, p.y + hh
        if d == "down":
            return p.x, p.y - hh
        if d == "left":
            return p.x - hw, p.y
        if d == "right":
            return p.x + hw, p.y
        return p.x, p.y

    # ---------------------------------------------------------- attributes

    def parse_optexp(self):
        k = self.peek().kind
        if k in ("NUM", "NAME", "ENV", "FUNC1", "FUNC2", "rand", "(", "-", "+", "!",
                 "LABEL", "Here", "last", "`"):
            return self.scalar(self.parse_expression())
        return None

    def parse_attributes(self, st: ObjState) -> None:
        while True:
            t = self.peek()
            k = t.kind
            if k in ("ht", "wid", "rad", "diam", "thick", "scaled"):
                self.next()
                getattr(self, "attr_" + k)(st, self.scalar(self.parse_expression()))
            elif k == "DIR":
                self.next()
                self.attr_direction(st, t.value, self.parse_optexp())
            elif k == "LINETYPE":
                self.next()
                self.attr_linetype(st, t.value, self.parse_optexp())
            elif k == "chop":
                self.next()
                self.attr_chop(st, self.parse_optexp())
            elif k == "fill":
                self.next()
                self.attr_fill(st, self.parse_optexp())
            elif k == "ARROW":
                self.next()
                self.attr_arrow(st, t.value, self.parse_optexp())
            elif k == "then":
                self.next()
                self.attr_then(st)
            elif k == "cw":
                self.next()
                self.attr_cw(st)
            elif k == "ccw":
                self.next()
                self.attr_ccw(st)
            elif k == "same":
                self.next()
                self.attr_same(st)
            elif k in ("STRING", "sprintf"):
                self.attr_text(st, self.parse_stringexpr())
            elif k == "by":
                self.next()
                x, y = self.parse_position()
                self.attr_by(st, x, y)
            elif k == "from":
                self.next()
                x, y = self.parse_position()
                self.attr_from(st, x, y)
            elif k == "to":
                self.next()
                x, y = self.parse_position()
                self.do_to(st, x, y)
            elif k == "at":
                self.next()
                st.at = self.parse_position()
                st.corner = "c"
                st.flags.add("at")
            elif k == "TEXTPOS":
                self.next()
                self.attr_textpos(st, t.value)
            elif k == "COLR":
                self.next()
                self.attr_color(st, t.value, self.parse_stringexpr())
            elif k == "with":
                self.next()
                self.parse_with(st)
            else:
                return

    def bad_attribute(self, st: ObjState, what: str) -> None:
        self.error(f"'{what}' is not valid for a {st.prim.kind}")

    def attr_ht(self, st: ObjState, v: float) -> None:
        p = st.prim
        k = p.kind
        at = "at" in st.flags
        if k in ("box", "block"):
            r = 0.5 * (v - p.h)
            p.h = v
            if not at:
                if p.direction == "up":
                    self.shift(p, 0.0, r)
                elif p.direction == "down":
                    self.shift(p, 0.0, -r)
        elif k == "string":
            p.h = v
        elif k == "circle":
            if not at:
                if p.direction == "up":
                    p.y += 0.5 * v - p.r
                elif p.direction == "down":
                    p.y += p.r - 0.5 * v
            p.r = 0.5 * v
        elif k == "ellipse":
            if not at:
                if p.direction == "up":
                    p.y += 0.5 * (v - p.h)
                elif p.direction == "down":
                    p.y += 0.5 * (p.h - v)
            p.h = v
        elif k in LINEAR or k == "arc":
            p.ah = v
        else:
            self.bad_attribute(st, "ht")

    def attr_wid(self, st: ObjState, v: float) -> None:
        p = st.prim
        k = p.kind
        at = "at" in st.flags
        if k in ("box", "block"):
            r = 0.5 * (v - p.w)
            p.w = v
            if not at:
                if p.direction == "right":
                    self.shift(p, r, 0.0)
                elif p.direction == "left":
                    self.shift(p, -r, 0.0)
        elif k == "string":
            p.w = v
        elif k == "circle":
            if not at:
                if p.direction == "right":
                    p.x += 0.5 * v - p.r
                elif p.direction == "left":
                    p.x += p.r - 0.5 * v
            p.r = 0.5 * v
        elif k == "ellipse":
            if not at:
                if p.direction == "right":
                    p.x += 0.5 * (v - p.w)
                elif p.direction == "left":
                    p.x += 0.5 * (p.w - v)
            p.w = v
        elif k in LINEAR or k == "arc":
            p.aw = v
        else:
            self.bad_attribute(st, "wid")

    def attr_rad(self, st: ObjState, v: float) -> None:
        p = st.prim
        k = p.kind
        if k == "box":
            p.brad = v
        elif k == "arc":
            end = self.arcend(p) if p.direction is None else None
            c, s = math.cos(p.sang), math.sin(p.sang)
            p.x += p.arad * c
            p.y += p.arad * s
            p.arad = v
            p.x -= p.arad * c
            p.y -= p.arad * s
            st.flags.add("radius")
            if end is not None:
                self.do_to(st, end[0], end[1])
        elif k == "circle":
            if "at" not in st.flags:
                if p.direction == "left":
                    p.x += p.r - v
                elif p.direction == "right":
                    p.x += v - p.r
                elif p.direction == "up":
                    p.y += v - p.r
                elif p.direction == "down":
                    p.y += p.r - v
            p.r = v
            st.flags.add("radius")
        else:
            self.bad_attribute(st, "rad")

    def attr_diam(self, st: ObjState, v: float) -> None:
        p = st.prim
        if p.kind != "circle":
            self.bad_attribute(st, "diam")
            return
        r = 0.5 * v
        if "at" not in st.flags:
            if p.direction == "left":
                p.x += p.r - r
            elif p.direction == "right":
                p.x += r - p.r
            elif p.direction == "up":
                p.y += r - p.r
            elif p.direction == "down":
                p.y += p.r - r
        p.r = r

    def attr_thick(self, st: ObjState, v: float) -> None:
        p = st.prim
        p.lthick = v if v >= 0.0 else self.venv(p, "linethick")

    def attr_scaled(self, st: ObjState, v: float) -> None:
        p = st.prim
        k = p.kind
        r = v - 1
        cx, cy = self.corner(p, "c")
        at = "at" in st.flags
        if k in ("box", "block", "string"):
            dx = dy = 0.0
            if not at:
                dx, dy = p.w * r / 2, p.h * r / 2
            self.scaleobj(p, v)
            d = p.direction
            if d == "right":
                self.shift(p, cx - p.x + dx, cy - p.y)
            elif d == "left":
                self.shift(p, cx - p.x - dx, cy - p.y)
            elif d == "up":
                self.shift(p, cx - p.x, cy - p.y + dy)
            elif d == "down":
                self.shift(p, cx - p.x, cy - p.y - dy)
        elif k in ("circle", "ellipse"):
            if k == "circle":
                p.r *= v
                hw = hh = p.r
            else:
                p.w *= v
                p.h *= v
                hw, hh = p.w / 2, p.h / 2
            if not at:
                d = p.direction
                p.x, p.y = cx, cy
                if d == "up":
                    p.y = cy + hh
                elif d == "down":
                    p.y = cy - hh
                elif d == "right":
                    p.x = cx + hw
                elif d == "left":
                    p.x = cx - hw
        elif k in LINEAR or k == "arc":
            self.scaleobj(p, v)
            nx, ny = self.corner(p, "c")
            self.shift(p, cx - nx, cy - ny)
        else:
            self.bad_attribute(st, "scaled")

    def lineardir(self, st: ObjState, dy: float, dx: float) -> None:
        p = st.prim
        seg = p.segs[-1]
        if not ("to" in st.flags or "dir" in st.flags):
            seg.x1, seg.y1 = seg.x0, seg.y0
        d = p.direction
        if d == "up":
            seg.y1 += dy
        elif d == "down":
            seg.y1 -= dy
        elif d == "left":
            seg.x1 -= dx
        elif d == "right":
            seg.x1 += dx
        st.flags.add("dir")

    def attr_direction(self, st: ObjState, d: str, v) -> None:
        p = st.prim
        if p.kind not in LINEAR and p.kind != "arc":
            self.bad_attribute(st, d)
            return
        self.envblock.direction = d
        env = self.env()
        if p.kind == "arc":
            c, s = math.cos(p.sang), math.sin(p.sang)
            p.x += p.arad * c
            p.y += p.arad * s
            turn = (p.direction, d)
            if turn in CCW_TURNS:
                p.aang = 0.5 * PI
            elif turn in CW_TURNS:
                p.aang = -0.5 * PI
            if v is not None:
                p.arad = v
            p.x -= p.arad * c
            p.y -= p.arad * s
            p.direction = d
            return
        p.direction = d
        if v is not None:
            r = s = v
        elif p.kind == "move":
            r, s = env["moveht"], env["movewid"]
        else:
            r, s = env["lineht"], env["linewid"]
        self.lineardir(st, r, s)

    def attr_linetype(self, st: ObjState, lt: str, v) -> None:
        p = st.prim
        p.spec = lt
        if v is None:
            if lt == "dashed":
                p.lparam = self.env()["dashwid"]
        elif lt in ("solid", "invis") and p.kind in LINEAR:
            self.lineardir(st, v, v)
        elif lt in ("solid", "invis"):
            self.bad_attribute(st, lt + " <expression>")
        else:
            p.lparam = v

    def attr_chop(self, st: ObjState, v) -> None:
        if st.prim.kind not in LINEAR:
            self.bad_attribute(st, "chop")
            return
        r = v if v is not None else self.env()["circlerad"]
        if "chop" in st.flags:
            st.endchop = r
        else:
            st.startchop = st.endchop = r
            st.flags.add("chop")

    def attr_fill(self, st: ObjState, v) -> None:
        p = st.prim
        value = v if v is not None else self.env()["fillval"]
        if p.kind in ("box", "circle", "ellipse", "arc") or p.kind in LINEAR:
            p.fill = value
        else:
            self.bad_attribute(st, "fill")

    def attr_arrow(self, st: ObjState, a: str, v) -> None:
        p = st.prim
        if p.kind not in ("spline", "arc", "arrow", "line"):
            self.bad_attribute(st, a)
            return
        num = rnd(v) if v is not None else rnd(self.env()["arrowhead"])
        if p.kind == "arc":
            p.atype, p.ahnum = a, num
        else:
            seg = p.segs[-1]
            seg.atype, seg.ahnum = a, num

    def attr_then(self, st: ObjState) -> None:
        p = st.prim
        if p.kind not in LINEAR:
            self.bad_attribute(st, "then")
            return
        last = p.segs[-1]
        seg = Seg(last.x1, last.y1, last.x1, last.y1)
        seg.atype, seg.ahnum = last.atype, last.ahnum
        p.segs.append(seg)
        st.flags = {f for f in st.flags if f in ("chop", "dir")}

    def attr_cw(self, st: ObjState) -> None:
        p = st.prim
        if p.kind != "arc":
            self.bad_attribute(st, "cw")
            return
        if p.aang > 0.0 and p.direction is None:
            p.aang = -abs(principal(2.0 * PI - p.aang, 2.0 * PI))
        elif p.direction is not None:
            p.x, p.y = self.arcstart(p)
            d = p.direction
            if d == "up":
                p.x += p.arad
            elif d == "down":
                p.x -= p.arad
            elif d == "left":
                p.y += p.arad
            elif d == "right":
                p.y -= p.arad
            if p.aang > 0.0:
                p.sang = principal(p.sang + PI, PI)
            p.aang = -abs(p.aang)
        st.flags.add("cw")

    def attr_ccw(self, st: ObjState) -> None:
        p = st.prim
        if p.kind != "arc":
            self.bad_attribute(st, "ccw")
            return
        if p.aang < 0.0 and p.direction is None:
            p.aang = abs(principal(p.aang - 2.0 * PI, 2.0 * PI))
        elif p.direction is not None:
            p.x, p.y = self.arcstart(p)
            d = p.direction
            if d == "up":
                p.x -= p.arad
            elif d == "down":
                p.x += p.arad
            elif d == "left":
                p.y -= p.arad
            elif d == "right":
                p.y += p.arad
            if p.aang < 0.0:
                p.sang = principal(p.sang + PI, PI)
            p.aang = abs(p.aang)
        st.flags.add("ccw")

    def attr_same(self, st: ObjState) -> None:
        p = st.prim
        q = self.nthprimobj(self.envblock.children, 0, p.kind)
        if q is None:
            self.error(f"there is no previous {p.kind} for 'same'")
            return
        p.lparam, p.lthick, p.spec = q.lparam, q.lthick, q.spec
        p.outline, p.shaded = q.outline, q.shaded
        d = p.direction
        if p.kind in ("box", "string"):
            if p.kind == "box":
                if d == "up":
                    p.y += 0.5 * (q.h - p.h)
                elif d == "down":
                    p.y -= 0.5 * (q.h - p.h)
                elif d == "left":
                    p.x -= 0.5 * (q.w - p.w)
                elif d == "right":
                    p.x += 0.5 * (q.w - p.w)
            p.fill, p.h, p.w, p.brad = q.fill, q.h, q.w, q.brad
        elif p.kind == "circle":
            if d == "up":
                p.y += q.r - p.r
            elif d == "down":
                p.y += p.r - q.r
            elif d == "left":
                p.x += p.r - q.r
            elif d == "right":
                p.x += q.r - p.r
            p.fill, p.r = q.fill, q.r
        elif p.kind == "ellipse":
            if d == "up":
                p.y += 0.5 * (q.h - p.h)
            elif d == "down":
                p.y -= 0.5 * (q.h - p.h)
            elif d == "left":
                p.x -= 0.5 * (q.w - p.w)
            elif d == "right":
                p.x += 0.5 * (q.w - p.w)
            p.fill, p.h, p.w = q.fill, q.h, q.w
        elif p.kind == "arc":
            x1, z1 = self.arcstart(p)
            p.arad, p.sang, p.aang = q.arad, q.sang, q.aang
            p.x = x1 - p.arad * math.cos(p.sang)
            p.y = z1 - p.arad * math.sin(p.sang)
        elif p.kind in LINEAR:
            seg, ref = p.segs[-1], q.segs[0]
            seg.x1 = seg.x0 + ref.x1 - ref.x0
            seg.y1 = seg.y0 + ref.y1 - ref.y0
            seg.atype, seg.ahnum = ref.atype, ref.ahnum
            p.ah, p.aw = q.ah, q.aw
        p.direction = q.direction

    def attr_text(self, st: ObjState, text: str) -> None:
        p = st.prim
        if p.kind in LINEAR:
            p.segs[-1].texts.append([text, 0])
            return
        count = len(p.texts)
        p.texts.append([text, 0])
        if p.kind == "string" and count > 0:
            p.h *= (count + 1) // count

    def attr_by(self, st: ObjState, x: float, y: float) -> None:
        p = st.prim
        if p.kind in LINEAR:
            seg = p.segs[-1]
            self.do_to(st, x + seg.x0, y + seg.y0)
        elif p.kind == "arc":
            sx, sy = self.arcstart(p)
            self.do_to(st, x + sx, y + sy)
        else:
            self.bad_attribute(st, "by")

    def attr_from(self, st: ObjState, x: float, y: float) -> None:
        p = st.prim
        if p.kind in LINEAR:
            if "to" not in st.flags:
                first = p.segs[0]
                self.shift(p, x - first.x0, y - first.y0)
            else:
                p.segs[-1].x0, p.segs[-1].y0 = x, y
            st.flags.add("from")
        elif p.kind == "arc":
            end = self.arcend(p) if "to" in st.flags else None
            p.x = x - p.arad * math.cos(p.sang)
            p.y = y - p.arad * math.sin(p.sang)
            if end is not None:
                self.do_to(st, end[0], end[1])
                return
            st.flags.add("from")
        else:
            self.bad_attribute(st, "from")

    def do_to(self, st: ObjState, x: float, y: float) -> None:
        p = st.prim
        if p.kind in LINEAR:
            if "to" in st.flags:
                self.attr_then(st)
            seg = p.segs[-1]
            seg.x1, seg.y1 = x, y
            st.flags.add("to")
            return
        if p.kind != "arc":
            self.bad_attribute(st, "to")
            return
        x1, z1 = self.arcstart(p)
        dx, dy = x - x1, y - z1
        ts = dx * dx + dy * dy
        d = p.direction if p.direction is not None else st.toklen
        if ts == 0.0:
            p.aang = 0.0
        else:
            t = math.sqrt(max(0.0, 4.0 * p.arad * p.arad - ts) / ts)
            if t <= 0.0:
                p.arad = 0.5 * math.sqrt(ts)
            if d == "up":
                if p.aang * (-dx - t * dy) < 0.0:
                    t = -t
            elif d == "down":
                if p.aang * (-dx - t * dy) > 0.0:
                    t = -t
            elif d == "right":
                if p.aang * (dy - t * dx) < 0.0:
                    t = -t
            elif d == "left":
                if p.aang * (dy - t * dx) > 0.0:
                    t = -t
            p.x = x1 + 0.5 * (dx + t * dy)
            p.y = z1 + 0.5 * (dy - t * dx)
            p.sang, p.aang = setangles(p.aang, p.x, p.y, x1, z1, x, y)
        if p.direction is not None:
            st.toklen = p.direction
        p.direction = None
        st.flags.add("to")

    def attr_textpos(self, st: ObjState, how: str) -> None:
        p = st.prim
        texts = p.segs[-1].texts if p.kind in LINEAR else p.texts
        if not texts:
            self.error(f"'{how}' needs a preceding string")
            return
        texts[-1][1] = setjust(texts[-1][1], how)

    def attr_color(self, st: ObjState, how: str, colour: str) -> None:
        p = st.prim
        k = p.kind
        outline_ok = k in ("spline", "arrow", "line", "arc", "ellipse", "circle", "box")
        shade_ok = k in ("ellipse", "circle", "box", "spline", "arrow", "line", "arc")
        if how == "shade":
            if shade_ok:
                p.shaded = colour
            else:
                self.bad_attribute(st, "shaded")
        elif how == "outline":
            if outline_ok:
                p.outline = colour
            else:
                self.bad_attribute(st, "outlined")
        elif outline_ok:
            p.outline = colour
            if k not in ("spline", "arrow", "line", "arc") and shade_ok:
                p.shaded = colour
        else:
            self.bad_attribute(st, "colored")

    def parse_with(self, st: ObjState) -> None:
        p = st.prim
        internal = None
        while self.peek().kind == ".":
            self.next()
            base = internal if internal is not None else p
            t = self.peek()
            if t.kind == "LABEL":
                self.next()
                name = t.value + self.parse_suffix()
                internal = self.findplace(base.children, name)
                if internal is None:
                    raise PicError(f"there is no place {name} in this object")
            else:
                nth = self.parse_nth(self.next())
                internal = self.nthprimobj(base.children, nth, self.parse_primobj())
                if internal is None:
                    raise PicError("the object named in 'with' was not found")
        t = self.peek()
        if t.kind == "CORNER":
            self.next()
            self.expect("at")
            st.at = self.parse_position()
            st.corner = t.value
        elif t.kind == "at":
            self.next()
            st.at = self.parse_position()
            st.corner = None
        else:
            pair = self.parse_position()
            self.expect("at")
            st.at = self.parse_position()
            if p.kind in ("arc", "ellipse", "circle", "block", "string", "box"):
                st.corner = "pair"
                st.pair = pair
            else:
                self.bad_attribute(st, "with (x,y)")
                st.corner = "c"
        st.internal = internal
        st.flags.add("at")

    # ---------------------------------------------------------- geometry queries

    def corner(self, p: Prim, which):
        k = p.kind
        if k in LINEAR:
            segs = p.segs
            x, y = segs[0].x0, segs[0].y0
            last = segs[-1]
            if which is None or which == "c":
                return 0.5 * (x + last.x1), 0.5 * (y + last.y1)
            if which == "start":
                return x, y
            if which == "end":
                return last.x1, last.y1
            for seg in segs:
                ex, ey = seg.x1, seg.y1
                if which == "n":
                    better = ey > y
                elif which == "s":
                    better = ey < y
                elif which == "e":
                    better = ex > x
                elif which == "w":
                    better = ex < x
                elif which == "ne":
                    better = (ey > y and ex >= x) or (ey >= y and ex > x)
                elif which == "se":
                    better = (ey < y and ex >= x) or (ey <= y and ex > x)
                elif which == "sw":
                    better = (ey < y and ex <= x) or (ey <= y and ex < x)
                elif which == "nw":
                    better = (ey > y and ex <= x) or (ey >= y and ex < x)
                else:
                    better = False
                if better:
                    x, y = ex, ey
            return x, y
        x, y = p.x, p.y
        if which is None and k not in ("string", "latex"):
            return x, y
        if k not in ("box", "string", "block", "circle", "ellipse", "arc"):
            return x, y
        if k in ("arc", "circle", "ellipse", "box") and which in ("ne", "nw", "se", "sw"):
            if k == "box":
                dy = min(p.brad, min(abs(p.h), abs(p.w)) / 2) * (1 - 1 / SQRT2)
                dx = p.w / 2 - dy
                dy = p.h / 2 - dy
            elif k == "ellipse":
                dx, dy = p.w * (0.5 / SQRT2), p.h * (0.5 / SQRT2)
            elif k == "arc":
                dx = dy = p.arad / SQRT2
            else:
                dx = dy = p.r / SQRT2
            if which == "se":
                dy = -dy
            elif which == "nw":
                dx = -dx
            elif which == "sw":
                dx, dy = -dx, -dy
            return x + dx, y + dy
        if k == "arc":
            if which == "n":
                return x, y + p.arad
            if which == "s":
                return x, y - p.arad
            if which == "e":
                return x + p.arad, y
            if which == "w":
                return x - p.arad, y
            if which == "start":
                return self.arcstart(p)
            if which == "end":
                return self.arcend(p)
            return x, y
        n, s, e, w = self.nesw_one(p)
        result = {"n": (x, n), "s": (x, s), "e": (e, y), "w": (w, y), "ne": (e, n),
                  "se": (e, s), "sw": (w, s), "nw": (w, n), "c": (x, y)}.get(which)
        if result is None:
            if which in ("start", "end"):
                self.error(f"'.{which}' is not defined for a {k}")
            return x, y
        return result

    def param(self, p: Prim, which: str) -> float:
        k = p.kind
        if which in ("ht", "wid"):
            if k in ("box", "string", "block", "ellipse"):
                return p.h if which == "ht" else p.w
            if k == "circle":
                return 2.0 * p.r
            if k in LINEAR:
                return p.ah if which == "ht" else p.aw
            return 0.0
        if which == "rad":
            if k == "circle":
                return p.r
            if k == "arc":
                return p.arad
            if k == "box":
                return p.brad
        elif which == "diam":
            if k == "circle":
                return 2.0 * p.r
            if k == "arc":
                return 2.0 * p.arad
        elif which == "thick":
            if k in ("arc", "spline", "arrow", "line", "circle", "ellipse", "box"):
                return self.venv(p, "linethick") if p.lthick == MDISTMAX else abs(p.lthick)
        elif which == "len":
            if k in LINEAR:
                first, last = p.segs[0], p.segs[-1]
                return math.hypot(last.x1 - first.x0, last.y1 - first.y0)
        self.error(f"'.{which}' is not defined for a {k}")
        return 0.0

    # ---------------------------------------------------------- expressions

    @staticmethod
    def scalar(v):
        if isinstance(v, tuple):
            raise PicError("a number was expected, not a position")
        if isinstance(v, str):
            raise PicError("a number was expected, not a string")
        return v

    def truth(self, v) -> bool:
        return self.scalar(v) != 0.0

    def parse_position(self, allow_scalar: bool = False, interp_lt: bool = True):
        v = self.parse_expression()
        if isinstance(v, tuple):
            return v
        v = self.scalar(v)
        k = self.peek().kind
        if k == "between" or k == "of":
            self.next()
            if k == "of":
                self.expect("the")
                self.expect("way")
                self.expect("between")
            a = self.parse_position()
            self.expect("and")
            b = self.parse_position()
            return a[0] + v * (b[0] - a[0]), a[1] + v * (b[1] - a[1])
        if k == "<" and interp_lt:
            self.next()
            return self.finish_interpolation(v, self.parse_position())
        if k == ",":
            self.next()
            return v, self.scalar(self.parse_expression())
        if allow_scalar:
            return v
        raise PicError(f"a position was expected near {self.peek()}")

    def finish_interpolation(self, v: float, a):
        """Complete v<a,b> [+/- location] after the first position has been parsed."""
        self.expect(",")
        b = self.parse_position()
        t = self.next()
        if not (t.kind == "CMP" and t.value == ">"):
            raise PicError(f"expected '>' but found {t}")
        x = a[0] + v * (b[0] - a[0])
        y = a[1] + v * (b[1] - a[1])
        while self.peek().kind in ("+", "-"):
            sign = 1.0 if self.next().kind == "+" else -1.0
            loc = self.parse_term()
            if not isinstance(loc, tuple):
                raise PicError("a position was expected")
            x += sign * loc[0]
            y += sign * loc[1]
        return x, y

    def parse_logic(self):
        v = self.parse_logand()
        while self.peek().kind == "||":
            self.next()
            w = self.parse_logand()
            v = 1.0 if (self.truth(v) or self.truth(w)) else 0.0
        return v

    def parse_logand(self):
        v = self.parse_compare()
        while self.peek().kind == "&&":
            self.next()
            w = self.parse_compare()
            v = 1.0 if (self.truth(v) and self.truth(w)) else 0.0
        return v

    def parse_compare(self):
        left = self.parse_compare_operand()
        while True:
            t = self.peek()
            if t.kind == "CMP":
                op = t.value
            elif t.kind == "<":
                op = "<"
            else:
                return left
            self.next()
            right = self.parse_compare_operand()
            if op == "<" and isinstance(right, tuple) and not isinstance(left, (tuple, str)) \
                    and self.peek().kind == ",":
                left = self.finish_interpolation(self.scalar(left), right)
                continue
            left = self.compare(left, op, right)

    def parse_compare_operand(self):
        if self.peek().kind in ("STRING", "sprintf"):
            return self.parse_stringexpr()
        return self.parse_position(allow_scalar=True, interp_lt=False)

    def compare(self, a, op: str, b) -> float:
        if isinstance(a, str) or isinstance(b, str):
            if not (isinstance(a, str) and isinstance(b, str)):
                self.error("a string can only be compared with a string")
                return 0.0
            c = (a > b) - (a < b)
        else:
            a, b = self.scalar(a), self.scalar(b)
            c = (a > b) - (a < b)
        return 1.0 if {"<": c < 0, "==": c == 0, "!=": c != 0, ">=": c >= 0,
                       "<=": c <= 0, ">": c > 0}[op] else 0.0

    def parse_expression(self):
        k = self.peek().kind
        if k == "+":
            self.next()
            v = self.parse_term()
        elif k == "-":
            self.next()
            v = self.parse_term()
            v = (-v[0], -v[1]) if isinstance(v, tuple) else -self.scalar(v)
        else:
            v = self.parse_term()
        while True:
            k = self.peek().kind
            if k not in ("+", "-"):
                return v
            self.next()
            w = self.parse_term()
            if isinstance(v, tuple) and isinstance(w, tuple):
                v = (v[0] + w[0], v[1] + w[1]) if k == "+" else (v[0] - w[0], v[1] - w[1])
            elif isinstance(v, tuple) or isinstance(w, tuple):
                raise PicError("a number and a position cannot be added")
            else:
                v = self.scalar(v) + self.scalar(w) if k == "+" else self.scalar(v) - self.scalar(w)

    def parse_term(self):
        v = self.parse_factor()
        while True:
            k = self.peek().kind
            if k not in ("*", "/", "%"):
                return v
            self.next()
            w = self.parse_factor()
            if k == "*":
                if isinstance(v, tuple):
                    f = self.scalar(w)
                    v = (v[0] * f, v[1] * f)
                elif isinstance(w, tuple):
                    f = self.scalar(v)
                    v = (w[0] * f, w[1] * f)
                else:
                    v = self.scalar(v) * self.scalar(w)
            elif k == "/":
                f = self.scalar(w)
                if f == 0.0:
                    self.error("division by zero")
                    if not isinstance(v, tuple):
                        v = 0.0
                elif isinstance(v, tuple):
                    v = (v[0] / f, v[1] / f)
                else:
                    v = self.scalar(v) / f
            else:
                i, j = rnd(self.scalar(v)), rnd(self.scalar(w))
                if j == 0:
                    self.error("division by zero")
                    v = 0.0
                else:
                    v = float(i - _c_trunc_div(i, j) * j)
            if isinstance(v, tuple) and self.peek().kind in ("DOTX", "DOTY"):
                # dpic: location * factor is a location, so (A*2).x is written A*2.x
                v = self.location_suffix(v)
                if self.peek().kind == "^":
                    self.next()
                    v = self.power(v, self.scalar(self.parse_factor()))

    def parse_factor(self):
        if self.peek().kind == "!":
            self.next()
            return 1.0 if self.scalar(self.parse_primary()) == 0.0 else 0.0
        v = self.parse_primary()
        if self.peek().kind == "^":
            self.next()
            e = self.scalar(self.parse_factor())
            return self.power(self.scalar(v), e)
        return v

    def power(self, x: float, e: float) -> float:
        if x == 0.0 and e < 0.0:
            self.error("division by zero")
            return x
        j = rnd(e)
        try:
            if j == e:
                return float(x ** j)
            if x < 0.0:
                self.error("negative number raised to a fractional power")
                return 0.0
            if x != 0.0:
                return math.exp(e * math.log(x))
        except OverflowError:
            self.error("arithmetic overflow")
            return DISTMAX
        return x

    def parse_primary(self):
        t = self.peek()
        k = t.kind
        if k == "NUM":
            if self.peek(1).kind == "NTH":
                return self.parse_place_value()
            self.next()
            return t.value
        if k == "ENV":
            self.next()
            return self.env()[t.value]
        if k == "NAME":
            self.next()
            return self.lookup_var(t.value + self.parse_suffix())
        if k == "FUNC1":
            self.next()
            self.expect("(")
            x = self.scalar(self.parse_expression())
            self.expect(")")
            return self.func1(t.value, x)
        if k == "FUNC2":
            self.next()
            self.expect("(")
            x = self.scalar(self.parse_expression())
            self.expect(",")
            y = self.scalar(self.parse_expression())
            self.expect(")")
            return self.func2(t.value, x, y)
        if k == "rand":
            self.next()
            self.expect("(")
            if self.peek().kind != ")":
                self.rng.seed(rnd(self.scalar(self.parse_expression())))
            self.expect(")")
            return self.rng.random()
        if k == "(":
            self.next()
            first = self.peek()
            if first.kind in ("NAME", "ENV") and self.peek(1).kind == "ASSIGN":
                value = self.parse_assignment()[0]
                while self.peek().kind == ",":
                    self.next()
                    value = self.parse_assignment()[0]
                self.expect(")")
                return value
            if first.kind == "sh":
                self.next()
                self.parse_stringexpr()
                self.expect(")")
                self.warn("sh command ignored: pppic does not run shell commands")
                return -1.0
            v = self.parse_logic()
            if self.peek().kind == ",":
                self.next()
                w = self.parse_logic()
                if not (isinstance(v, tuple) and isinstance(w, tuple)):
                    raise PicError("(position, position) expects two positions")
                v = (v[0], w[1])
            self.expect(")")
            return self.location_suffix(v)
        if k in ("LABEL", "Here", "last", "CORNER", "`", "{"):
            return self.parse_place_value()
        raise PicError(f"syntax error near {t}")

    def func1(self, name: str, x: float) -> float:
        try:
            if name == "abs":
                return abs(x)
            if name in ("acos", "asin"):
                if abs(x) > 1.0:
                    self.error(f"{name} argument out of range")
                    return 0.0
                r = math.sqrt(1 - x * x)
                return math.atan2(r, x) if name == "acos" else math.atan2(x, r)
            if name == "cos":
                return math.cos(x)
            if name == "sin":
                return math.sin(x)
            if name == "tan":
                c = math.cos(x)
                if c == 0.0:
                    self.error("tan argument out of range")
                    return DISTMAX
                return math.sin(x) / c
            if name == "exp":
                return math.exp(math.log(10.0) * x)
            if name == "expe":
                return math.exp(x)
            if name == "int":
                return float(int(x))
            if name in ("log", "loge"):
                if x <= 0.0:
                    self.error(f"{name} argument must be positive")
                    return 0.0
                return math.log10(x) if name == "log" else math.log(x)
            if name == "sign":
                return 1.0 if x >= 0.0 else -1.0
            if name == "sqrt":
                if x < 0.0:
                    self.error("sqrt of a negative number")
                    return 0.0
                return math.sqrt(x)
            if name == "floor":
                return float(math.floor(x))
        except OverflowError:
            self.error("arithmetic overflow")
            return DISTMAX
        raise PicError(f"unknown function {name}")

    def func2(self, name: str, x: float, y: float) -> float:
        if name == "atan2":
            return math.atan2(x, y)
        if name == "max":
            return max(x, y)
        if name == "min":
            return min(x, y)
        if y == 0.0:
            self.error("pmod with zero modulus")
            return 0.0
        value = x - y * math.floor(x / y)
        if value < 0.0:
            value += abs(y)
        return value

    def location_suffix(self, v):
        k = self.peek().kind
        if k in ("DOTX", "DOTY"):
            if not isinstance(v, tuple):
                raise PicError("'.x' and '.y' need a position")
            self.next()
            return v[0] if k == "DOTX" else v[1]
        return v

    def parse_place_value(self):
        t = self.peek()
        if t.kind == "Here":
            self.next()
            here = self.envblock.here
            return self.location_suffix((here[0], here[1]))
        if t.kind == "CORNER":
            self.next()
            if self.peek().kind == "of":
                self.next()
            p = self.parse_placename()
            return self.location_suffix(self.corner(p, t.value))
        p = self.parse_placename()
        t2 = self.peek()
        if t2.kind == "CORNER":
            self.next()
            return self.location_suffix(self.corner(p, t2.value))
        if t2.kind == "PARAM":
            self.next()
            return self.param(p, t2.value)
        return self.location_suffix(self.corner(p, None))

    def parse_placename(self) -> Prim:
        t = self.next()
        if t.kind == "LABEL":
            name = t.value + self.parse_suffix()
            p = self.find_label(name)
            if p is None:
                raise PicError(f"there is no place named {name}")
        else:
            nth = self.parse_nth(t)
            typ = self.parse_primobj()
            p = self.nthprimobj(self.envblock.children, nth, typ)
            if p is None:
                raise PicError(f"the requested {typ} does not exist")
        while self.peek().kind == ".":
            self.next()
            t = self.peek()
            if t.kind == "LABEL":
                self.next()
                name = t.value + self.parse_suffix()
                q = self.findplace(p.children, name)
                if q is None:
                    raise PicError(f"there is no place {name} in the object")
            else:
                nth = self.parse_nth(self.next())
                typ = self.parse_primobj()
                q = self.nthprimobj(p.children, nth, typ)
                if q is None:
                    raise PicError(f"the requested {typ} does not exist")
            p = q
        return p

    def parse_nth(self, t: Tok) -> int:
        if t.kind == "last":
            return 0
        if t.kind == "NUM":
            n = t.value
        elif t.kind == "`":
            n = self.scalar(self.parse_expression())
            self.expect("'")
        elif t.kind == "{":
            n = self.scalar(self.parse_expression())
            self.expect("}")
        else:
            raise PicError(f"an ordinal or 'last' was expected, found {t}")
        self.expect("NTH")
        if n <= 0:
            self.error("ordinal must be positive")
        k = rnd(n)
        if self.peek().kind == "last":
            self.next()
            k = -k
        return k

    def parse_primobj(self) -> str:
        t = self.next()
        if t.kind == "PRIM":
            return t.value
        if t.kind == "[]":
            return "block"
        if t.kind == "[":
            self.expect("]")
            return "block"
        if t.kind == "STRING":
            return "string"
        raise PicError(f"an object type was expected, found {t}")

    # ---------------------------------------------------------- strings

    def parse_stringexpr(self) -> str:
        s = self.parse_string()
        while self.peek().kind == "+" and self.peek(1).kind in ("STRING", "sprintf"):
            self.next()
            s += self.parse_string()
        return s

    def parse_string(self) -> str:
        t = self.next()
        if t.kind == "STRING":
            return t.value
        if t.kind == "sprintf":
            self.expect("(")
            fmt = self.parse_stringexpr()
            args = []
            while self.peek().kind == ",":
                self.next()
                args.append(self.scalar(self.parse_expression()))
            self.expect(")")
            return self.sprintf(fmt, args)
        raise PicError(f"a string was expected, found {t}")

    def sprintf(self, fmt: str, args: list[float]) -> str:
        out = []
        i = 0
        used = 0
        n = len(fmt)
        while i < n:
            c = fmt[i]
            if c != "%":
                out.append(c)
                i += 1
                continue
            if fmt.startswith("%%", i):
                out.append("%")
                i += 2
                continue
            m = _SPRINTF_RE.match(fmt, i)
            if m is None:
                self.error("bad sprintf format")
                out.append(fmt[i:])
                break
            if used >= len(args):
                self.error("too few sprintf arguments")
                break
            out.append(("%" + m.group(1) + m.group(2) + m.group(3)) % args[used])
            used += 1
            i = m.end()
        if used < len(args):
            self.error("too many sprintf arguments")
        return "".join(out)

    # ---------------------------------------------------------- bounding boxes

    def getnesw(self, prims):
        b = [DISTMAX, -DISTMAX, DISTMAX, -DISTMAX]
        for p in prims:
            self.nesw(p, b)
        s, n, w, e = b
        if s > n:
            s = n = 0.0
        if w > e:
            w = e = 0.0
        return n, s, e, w

    def nesw_one(self, p: Prim):
        b = [DISTMAX, -DISTMAX, DISTMAX, -DISTMAX]
        self.nesw(p, b)
        return b[1], b[0], b[3], b[2]

    def nesw(self, p: Prim, b: list) -> None:
        k = p.kind
        if k in ("box", "block", "circle", "ellipse"):
            if k == "circle":
                hw = hh = p.r
            else:
                hw, hh = p.w * 0.5, p.h * 0.5
            b[1] = max(b[1], p.y + hh)
            b[0] = min(b[0], p.y - hh)
            b[2] = min(b[2], p.x - hw)
            b[3] = max(b[3], p.x + hw)
        elif k == "string":
            just = p.texts[0][1] if p.texts else 0
            off = self.venv(p, "textoffset")
            ht, wd = p.h, p.w
            y = p.y
            if just & JUST_A:
                y += ht * 0.5 + off
            elif just & JUST_B:
                y += -ht * 0.5 - off
            x = p.x
            if just & JUST_R:
                x += -wd * 0.5 - off
            elif just & JUST_L:
                x += wd * 0.5 + off
            b[1] = max(b[1], y + ht * 0.5)
            b[0] = min(b[0], y - ht * 0.5)
            b[2] = min(b[2], x - wd * 0.5)
            b[3] = max(b[3], x + wd * 0.5)
        elif k in LINEAR:
            for seg in p.segs:
                self.nesw_line(p, seg, b)
        elif k == "arc":
            sang = principal(p.sang, PI)
            eang = p.sang + p.aang
            r = p.arad
            if _inarc(sang, eang, 0.5 * PI, p.aang):
                b[1] = max(b[1], p.y + r)
            else:
                b[1] = max(b[1], p.y + r * max(math.sin(sang), math.sin(eang)))
            if _inarc(sang, eang, -0.5 * PI, p.aang):
                b[0] = min(b[0], p.y - r)
            else:
                b[0] = min(b[0], p.y + r * min(math.sin(sang), math.sin(eang)))
            if _inarc(sang, eang, PI, p.aang):
                b[2] = min(b[2], p.x - r)
            else:
                b[2] = min(b[2], p.x + r * min(math.cos(sang), math.cos(eang)))
            if _inarc(sang, eang, 0.0, p.aang):
                b[3] = max(b[3], p.x + r)
            else:
                b[3] = max(b[3], p.x + r * max(math.cos(sang), math.cos(eang)))

    def nesw_line(self, p: Prim, seg: Seg, b: list) -> None:
        b[2] = min(b[2], seg.x0, seg.x1)
        b[3] = max(b[3], seg.x0, seg.x1)
        b[0] = min(b[0], seg.y0, seg.y1)
        b[1] = max(b[1], seg.y0, seg.y1)
        if not seg.atype:
            return
        awd = p.aw if p.aw != 0.0 else self.venv(p, "arrowht")
        aht = p.ah if p.ah != 0.0 else self.venv(p, "arrowwid")
        heads = []
        if seg.atype in ("<-", "<->"):
            heads.append((seg.x0, seg.y0, affang(seg.x1, seg.y1, seg.x0, seg.y0)))
        if seg.atype in ("->", "<->"):
            heads.append((seg.x1, seg.y1, affang(seg.x0, seg.y0, seg.x1, seg.y1)))
        for ox, oy, cs in heads:
            for side in (0.5, -0.5):
                hx, hy = affine(aht, awd * side, ox, oy, cs)
                b[2] = min(b[2], hx)
                b[3] = max(b[3], hx)
                b[0] = min(b[0], hy)
                b[1] = max(b[1], hy)

    # ---------------------------------------------------------- PSTricks output

    def wcoord(self, x: float, y: float) -> str:
        return "(" + wfloat(x / self.fsc) + "," + wfloat(y / self.fsc) + ")"

    def getscale(self, xv: float, yv: float, n: float, s: float, e: float, w: float) -> None:
        env = self.gb.env
        sfact = env["scale"] if env["scale"] > 0.0 else 1.0
        xsc = sfact
        if e > w and env["maxpswid"] > 0.0 and (e - w) / sfact > env["maxpswid"]:
            xsc = (e - w) / env["maxpswid"]
            self.warn("picture wider than maxpswid; it has been scaled down")
        if n > s and env["maxpsht"] > 0.0 and (n - s) / sfact > env["maxpsht"]:
            xsc = max(xsc, (n - s) / env["maxpsht"])
            self.warn("picture taller than maxpsht; it has been scaled down")
        if xv > 0.0 and e > w:
            xsc = (e - w) / xv
        if yv > 0.0 and n > s and (xv == 0.0 or (n - s) / xsc > yv):
            xsc = (n - s) / yv
        self.scale = sfact
        self.fsc = xsc

    def draw_picture(self, xv: float, yv: float) -> str:
        n, s, e, w = self.getnesw(self.gb.children)
        self.getscale(xv, yv, n, s, e, w)
        self.out = out = []
        out.append("\\psset{unit=1in,cornersize=absolute,dimen=middle}%\n")
        out.append("\\begin{pspicture}" + self.wcoord(w, s) + self.wcoord(e, n) + "%\n")
        out.append(f"% {VERSION} option -p for PSTricks 0.93a or later\n")
        for p in self.gb.children:
            self.draw(p)
        out.append("\\end{pspicture}%\n")
        return "".join(out)

    def drawn(self, p: Prim, fill: float) -> bool:
        if 0.0 <= fill <= 1.0 or p.spec in ("dotted", "dashed", "solid"):
            return True
        return p.shaded is not None

    def line_options(self, p: Prim, spec: str, sep: str, atype: str, ahnum: int) -> str:
        out = []
        if p.lthick >= 0.0:
            out.append(sep + "linewidth=" + wfloat(p.lthick) + "pt")
            sep = ","
        if p.outline is not None:
            out.append(sep + "linecolor=" + p.outline)
            sep = ","
        if p.kind == "block":
            spec = "invis"
        if spec in ("invis", "dotted", "dashed"):
            out.append(sep + "%\nlinestyle=")
            if spec == "dashed":
                out.append("dashed")
                if p.lparam >= 0.0:
                    dash = wfloat(p.lparam / self.fsc)
                    out.append(",dash=" + dash + " " + dash)
            elif spec == "dotted":
                out.append("dotted")
                if p.lparam >= 0.0:
                    out.append(",dotsep=" + wfloat(p.lparam / self.fsc))
            else:
                out.append("none")
            sep = ","
        if p.kind in ("spline", "line", "arrow", "arc") and atype and ahnum not in (0, 3):
            length = wfloat(0.0) if p.aw == 0.0 else wfloat(p.ah / p.aw)
            out.append(sep + "%\narrowsize=" + wfloat(p.aw / self.scale) + "in 0,arrowlength="
                       + length + ",arrowinset=0")
            sep = ","
        if sep == ",":
            out.append("]")
        return "".join(out)

    def fill_options(self, p: Prim, angle: float, shade, fill: float, atype: str = "", ahnum: int = 1) -> str:
        fill = math.floor(fill * 1000000 + 0.5) / 1000000.0
        out = []
        if 0.0 < fill < 1.0 and fill != self.lastfillval and fill not in (0.25, 0.5, 0.75):
            self.lastfillval = fill
            out.append("\\newgray{fillval}{" + wfloat(fill) + "}\n")
        k = p.kind
        if k in ("box", "block"):
            out.append("\\psframe")
        elif k == "ellipse":
            out.append("\\psellipse")
        elif k == "circle":
            out.append("\\pscircle")
        elif k == "spline":
            out.append("\\psbezier")
        elif k in ("line", "arrow"):
            out.append("\\psline")
        elif k == "arc":
            out.append("\\psarc" if angle >= 0.0 else "\\psarcn")
        if shade is not None:
            out.append("[fillstyle=solid,fillcolor=" + shade)
            sep = ","
        elif 0.0 <= fill <= 1.0:
            name = {0.0: "black", 0.25: "darkgray", 0.5: "gray", 0.75: "lightgray", 1.0: "white"}
            out.append("[fillstyle=solid,fillcolor=" + name.get(fill, "fillval"))
            sep = ","
        else:
            sep = "["
        if k == "box" and p.brad != 0.0:
            out.append(sep + "linearc=" + wfloat(p.brad / self.fsc))
            sep = ","
        out.append(self.line_options(p, p.spec, sep, atype, ahnum))
        return "".join(out)

    def write_text(self, p: Prim, texts, x: float, y: float) -> None:
        if not texts:
            return
        toff = self.venv(p, "textoffset") / self.scale * 72
        if len(texts) > 1:
            parts = []
            for text, just in texts:
                if just & JUST_L:
                    parts.append("\\rlap{\\hbox to " + wfloat(toff) + "bp{}" + text + "}")
                elif just & JUST_R:
                    parts.append("\\llap{" + text + "\\hbox to " + wfloat(toff) + "bp{}}")
                else:
                    parts.append(text)
            self.out.append("\\rput" + self.wcoord(x, y) + "{\\shortstack{" + "\\\\%\n".join(parts) + "}}\n")
            return
        text, just = texts[0]
        if just & (JUST_A | JUST_B | JUST_L | JUST_R):
            where = ("d" if just & JUST_B else "u" if just & JUST_A else "") + \
                    ("r" if just & JUST_L else "l" if just & JUST_R else "")
            self.out.append("\\uput{" + wfloat(toff) + "bp}[" + where + "]")
        else:
            self.out.append("\\rput")
        self.out.append(self.wcoord(x, y) + "{" + text + "}\n")

    def warc(self, cx: float, cy: float, ax: float, ay: float, bx: float, by: float, ccw: float) -> str:
        radius = math.hypot(bx - cx, by - cy)
        start = math.atan2(ay - cy, ax - cx)
        end = math.atan2(by - cy, bx - cx)
        if ccw > 0 and end < start:
            end += 2 * PI
        elif ccw < 0 and end > start:
            end -= 2 * PI
        return (self.wcoord(cx, cy) + "{" + wfloat(radius / self.fsc) + "}{"
                + wfloat(math.degrees(start)) + "}{" + wfloat(math.degrees(end)) + "}\n")

    def arrowhead(self, px, py, sx, sy, ht, wid, lth, typ, colour):
        """Draw a dpic-style arrowhead (arrowhead=0 or 3) and return the adjusted tip."""
        ltu = lth / 72 * self.scale
        cs = affang(sx, sy, px, py)
        po = 0.0 if wid == 0.0 else ltu * math.sqrt(ht * ht + wid * wid / 4) / wid
        po = min(po, ht)
        P = affine(po, 0.0, px, py, cs)
        h = ht - ltu / 2
        x = h - po
        v = 0.0 if ht == 0.0 else (wid / 2) * x / ht
        R = affine(h, v, px, py, cs)
        L = affine(h, -v, px, py, cs)
        t = 1.0 if x == 0.0 else ht / x
        Rx = (px + (R[0] - P[0]) * t, py + (R[1] - P[1]) * t)
        Lx = (px + (L[0] - P[0]) * t, py + (L[1] - P[1]) * t)
        Px = ((px + Lx[0] + Rx[0]) / 3, (py + Lx[1] + Rx[1]) / 3)
        if math.hypot(Px[0] - px, Px[1] - py) < math.hypot(P[0] - px, P[1] - py):
            Px = P
        y = 0.0 if ht == 0.0 else ht - po + ltu * wid / ht / 4
        colour = colour or "black"

        def prop(a, b, wa, wb, c):
            if c == 0.0:
                return self.wcoord(b[0], b[1])
            return self.wcoord((wa * a[0] + wb * b[0]) / c, (wa * a[1] + wb * b[1]) / c)

        if typ == 0:
            self.out.append("\\pscustom[linewidth=" + wfloat(lth) + "pt,linecolor=" + colour + "]{%\n"
                            + "\\psline" + prop(P, L, x - y, y, x) + self.wcoord(*P)
                            + prop(P, R, x - y, y, x) + "}\n")
        else:
            self.out.append("\\pscustom[linewidth=0pt,fillstyle=solid,fillcolor=" + colour
                            + ",linecolor=" + colour + "]{%\n\\psline" + self.wcoord(*Px)
                            + self.wcoord(*Rx) + self.wcoord(px, py) + self.wcoord(*Lx) + "\\closepath}%\n")
        return P

    def draw(self, p: Prim) -> None:
        k = p.kind
        out = self.out
        if k in ("box", "block"):
            fill = p.fill if k == "box" else -1.0
            if self.drawn(p, fill):
                n, s, e, w = self.nesw_one(p)
                out.append(self.fill_options(p, 0.0, p.shaded, fill) + self.wcoord(w, s)
                           + self.wcoord(e, n) + "\n")
            self.write_text(p, p.texts, p.x, p.y)
            if k == "block":
                for child in p.children:
                    self.draw(child)
        elif k == "ellipse":
            if self.drawn(p, p.fill):
                out.append(self.fill_options(p, 0.0, p.shaded, p.fill) + self.wcoord(p.x, p.y)
                           + self.wcoord(0.5 * p.w, 0.5 * p.h) + "\n")
            self.write_text(p, p.texts, p.x, p.y)
        elif k == "circle":
            if self.drawn(p, p.fill):
                out.append(self.fill_options(p, 0.0, p.shaded, p.fill) + self.wcoord(p.x, p.y)
                           + "{" + wfloat(p.r / self.fsc) + "}\n")
            self.write_text(p, p.texts, p.x, p.y)
        elif k == "arc":
            self.draw_arc(p)
        elif k in ("line", "arrow"):
            self.draw_line(p)
        elif k == "move":
            for seg in p.segs:
                if seg.texts:
                    self.write_text(p, seg.texts, 0.5 * (seg.x0 + seg.x1), 0.5 * (seg.y0 + seg.y1))
        elif k == "spline":
            self.draw_spline(p)
        elif k == "string":
            self.write_text(p, p.texts, p.x, p.y)
        elif k == "latex":
            if p.text is not None:
                out.append(p.text + "\n")
            elif p.lthick >= 0.0:
                out.append("\\psset{linewidth=" + wfloat(p.lthick) + "pt}%\n")

    def draw_arc(self, p: Prim) -> None:
        if self.drawn(p, p.fill):
            ax, ay = self.arcstart(p)
            bx, by = self.arcend(p)
            if p.shaded is not None or 0.0 <= p.fill <= 1.0:
                saved = p.lthick
                p.lthick = 0.0
                self.out.append(self.fill_options(p, p.aang, p.shaded, p.fill)
                                + self.warc(p.x, p.y, ax, ay, bx, by, p.aang))
                p.lthick = saved
            text = self.fill_options(p, p.aang, None, -1.0, p.atype, p.ahnum)
            if p.atype and p.ahnum not in (0, 3):
                text += "{" + p.atype + "}"
            self.out.append(text + self.warc(p.x, p.y, ax, ay, bx, by, p.aang))
        self.write_text(p, p.texts, p.x, p.y)

    def draw_line(self, p: Prim) -> None:
        segs = p.segs
        last = segs[-1]
        if self.drawn(p, p.fill):
            if p.shaded is not None or 0.0 <= p.fill <= 1.0:
                saved = p.lthick
                p.lthick = 0.0
                parts = [self.fill_options(p, 0.0, p.shaded, p.fill), self.wcoord(segs[0].x0, segs[0].y0)]
                parts += [self.wcoord(seg.x1, seg.y1) + "\n" for seg in segs]
                self.out.append("".join(parts))
                p.lthick = saved
            atype, ahnum = last.atype, last.ahnum
            if atype and ahnum in (0, 3):
                ht = self.qenv(p, "arrowht", p.ah)
                wid = self.qenv(p, "arrowwid", p.aw)
                lth = self.qenv(p, "linethick", p.lthick)
                if atype in ("<-", "<->"):
                    first = segs[0]
                    first.x0, first.y0 = self.arrowhead(first.x0, first.y0, first.x1, first.y1,
                                                        ht, wid, lth, ahnum, p.outline)
                if atype in ("->", "<->"):
                    last.x1, last.y1 = self.arrowhead(last.x1, last.y1, last.x0, last.y0,
                                                      ht, wid, lth, ahnum, p.outline)
            text = self.fill_options(p, 0.0, None, p.fill, atype, ahnum)
            if atype and ahnum not in (0, 3):
                text += "{" + atype + "}"
            parts = [text, self.wcoord(segs[0].x0, segs[0].y0)]
            parts += [self.wcoord(seg.x1, seg.y1) + "\n" for seg in segs]
            self.out.append("".join(parts))
        for seg in segs:
            if seg.texts:
                self.write_text(p, seg.texts, 0.5 * (seg.x1 + seg.x0), 0.5 * (seg.y0 + seg.y1))

    def draw_spline(self, p: Prim) -> None:
        segs = p.segs
        last = segs[-1]
        total = len(segs)
        if p.shaded is not None or 0.0 <= p.fill <= 1.0:
            saved = p.lthick
            p.lthick = 0.0
            self.out.append(self.fill_options(p, 0.0, p.shaded, p.fill) + "%\n")
            p.lthick = saved
            for index, seg in enumerate(segs):
                self.spline_segment(p, seg, total - index, total)
        atype, ahnum = last.atype, last.ahnum
        if atype and ahnum in (0, 3):
            ht = self.qenv(p, "arrowht", p.ah)
            wid = self.qenv(p, "arrowwid", p.aw)
            lth = self.qenv(p, "linethick", p.lthick)
            if atype in ("->", "<->"):
                last.x1, last.y1 = self.arrowhead(last.x1, last.y1, last.x0, last.y0, ht, wid, lth, ahnum, p.outline)
            if atype in ("<-", "<->"):
                first = segs[0]
                first.x0, first.y0 = self.arrowhead(first.x0, first.y0, first.x1, first.y1, ht, wid, lth, ahnum, p.outline)
        if p.spec == "invis":
            return
        text = self.fill_options(p, 0.0, None, p.fill, atype, ahnum)
        if atype and ahnum not in (0, 3):
            text += "{" + atype + "}"
        self.out.append(text + "%\n")
        for index, seg in enumerate(segs):
            self.spline_segment(p, seg, total - index, total)

    def spline_segment(self, p: Prim, seg: Seg, count: int, total: int) -> None:
        a0 = (seg.x0, seg.y0)
        a1 = (seg.x1, seg.y1)

        def prop(wa, wb, c):
            return self.wcoord((wa * a0[0] + wb * a1[0]) / c, (wa * a0[1] + wb * a1[1]) / c)

        a = p.tension
        if a == MDISTMAX:
            if count == total and count > 1:
                parts = [self.wcoord(*a0), prop(5, 1, 6), prop(4, 2, 6), prop(1, 1, 2), prop(1, 5, 6)]
            elif count > 1:
                parts = [prop(5, 1, 6), prop(1, 1, 2), prop(1, 5, 6)]
            else:
                parts = [prop(5, 1, 6), prop(1, 1, 2), prop(2, 4, 6), prop(1, 5, 6), self.wcoord(*a1)]
        elif count == total and count > 1:
            parts = [self.wcoord(*a0), prop(1 - a, a, 1)]
        elif count > 1:
            parts = [prop(1 + a, 1 - a, 2), prop(1, 1, 2), prop(1 - a, 1 + a, 2)]
        elif total == 1:
            parts = [prop(a, 1 - a, 1), prop(1 - a, a, 1), self.wcoord(*a1)]
        else:
            parts = [prop(a, 1 - a, 1), self.wcoord(*a1)]
        self.out.append("".join(parts) + "\n")


def _c_trunc_div(i: int, j: int) -> int:
    q = abs(i) // abs(j)
    return q if (i >= 0) == (j >= 0) else -q


def _inarc(start: float, fin: float, ang: float, arcang: float) -> bool:
    if arcang >= 0.0:
        while fin < start:
            fin += 2.0 * PI
        while ang < start:
            ang += 2.0 * PI
        return ang <= fin
    while fin > start:
        fin -= 2.0 * PI
    while ang > start:
        ang -= 2.0 * PI
    return ang >= fin


def compile_pic(source: str, *, base_dir=None, include_dirs=None) -> str:
    """Compile PIC source to PSTricks; raise PicError if any statement failed."""
    compiler = PicCompiler(base_dir=base_dir, include_dirs=include_dirs)
    output = compiler.compile(source)
    if compiler.errors:
        raise PicError("; ".join(compiler.errors[:5]))
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="pppic", description="dpic-compatible PIC processor (PSTricks output).")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__} (dpic 2025.08.01 compatible)")
    parser.add_argument("input", nargs="?", type=Path, help="PIC input file (default: stdin)")
    parser.add_argument("-p", action="store_true", help="PSTricks output (the only mode; accepted for dpic compatibility)")
    parser.add_argument("-o", "--output", type=Path, help="write the output to this file")
    parser.add_argument("-I", "--include", dest="include_dirs", action="append", type=Path, default=[],
                        help="directory searched by 'copy'")
    args = parser.parse_args(argv)
    try:
        if args.input is None:
            source, base = sys.stdin.read(), Path.cwd()
        else:
            data = args.input.read_bytes()
            try:
                source = data.decode("utf-8")
            except UnicodeDecodeError:
                source = data.decode("cp1252")
            base = args.input.resolve().parent
    except OSError as error:
        sys.stderr.write(f"pppic: {error}\n")
        return 1
    compiler = PicCompiler(base_dir=base, include_dirs=args.include_dirs)
    try:
        output = compiler.compile(source)
    except PicError as error:
        sys.stderr.write(f"pppic: {error}\n")
        return 1
    for message in compiler.messages:
        sys.stderr.write(message + "\n")
    for message in compiler.warnings:
        sys.stderr.write(f"pppic warning: {message}\n")
    for message in compiler.errors:
        sys.stderr.write(f"pppic error: {message}\n")
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    else:
        sys.stdout.write(output)
    return 1 if compiler.errors else 0


if __name__ == "__main__":
    raise SystemExit(main())








