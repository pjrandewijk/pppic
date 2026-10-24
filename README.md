# pppic

[![CI](https://github.com/pjrandewijk/pppic/actions/workflows/ci.yml/badge.svg)](https://github.com/pjrandewijk/pppic/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/pppic?label=PyPI)](https://pypi.org/project/pppic/)

A PIC processor written in pure Python that is compatible with
[dpic](https://ctan.org/pkg/dpic) 2025.08.01 and produces PSTricks output,
like `dpic -p`. No compiler, no `dpic.exe`, no dependencies.

The name follows gpic and dpic; the *pp* stands for **P**ython & **P**STricks.

pppic implements the dpic language: macros (`define`), `for` loops,
`if`/`else`, blocks, labels, positions, `copy`, `sprintf` and the object
geometry and PSTricks backend of dpic. The `sh` command is never executed.

It is one part of a pure-Python toolchain for
[Circuit Macros](https://ctan.org/pkg/circuit-macros):

| Package | Replaces | Role |
|---|---|---|
| [pym4](https://github.com/pjrandewijk/pym4) | GNU m4 | m4 → pic |
| **pppic** | dpic -p | pic → PSTricks |
| [m4cmpy](https://github.com/pjrandewijk/m4cmpy) | m4cm.py | m4 → PDF / PNG / JPG / SVG / EPS |

## Install

```
pip install pppic
```

## Use

Command line (`-p` is accepted for dpic compatibility; PSTricks is the only output):

```
pppic -p examples/shapes.pic > shapes.tex
pym4 -I path/to/circuit_macros circuit.m4 | pppic -p > circuit.tex
```

Python:

```python
from pppic import PicCompiler, compile_pic

print(compile_pic('.PS\nbox "hello"; arrow; circle\n.PE\n'))

compiler = PicCompiler(include_dirs=["path/to/circuit_macros"])
tex = compiler.compile(open("circuit.pic").read())
print(compiler.errors, compiler.warnings)
```

Include the result in LaTeX with `\usepackage{pstricks}` and compile with
`latex` + `dvips` (or use [m4cmpy](https://github.com/pjrandewijk/m4cmpy)).

## Examples and tests

[examples](examples) holds plain PIC files and a Circuit Macros diagram
pre-expanded by pym4. The unit tests also compile every example:

```
python -m unittest discover -s tests -v
```

## Licence

BSD-2-Clause; see [LICENSE](LICENSE). pppic is a Python port of dpic by
J. D. Aplevich, and keeps dpic's licence and copyright notice.
