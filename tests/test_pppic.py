import re
import tempfile
import unittest
from pathlib import Path

from pppic import PicCompiler, compile_pic, main


def pst(source: str) -> str:
    return compile_pic(".PS\n" + source + "\n.PE\n")


class CompilePicTests(unittest.TestCase):
    def test_basic_shapes_follow_dpic_pstricks_output(self):
        out = pst('line from (0,0) to (1,0) ->\nbox wid 0.4 ht 0.2 at (0.5,0.3)\n'
                  'circle rad 0.1 at (0.5,0.3)\n"hello" at (0.5,0.5)')
        self.assertIn(r"\begin{pspicture}(0,-0.025)(1,0.5)", out)
        self.assertIn("\\psline[%\narrowsize=0.05in 0,arrowlength=2,arrowinset=0]{->}(0,0)(1,0)", out)
        self.assertIn(r"\psframe(0.3,0.2)(0.7,0.4)", out)
        self.assertIn(r"\pscircle(0.5,0.3){0.1}", out)
        self.assertIn(r"\rput(0.5,0.5){hello}", out)

    def test_macros_and_loops(self):
        out = pst("define sq {box wid $1 ht $1 at ($2,0)}\nfor i=1 to 3 do { sq(0.1, i) }")
        self.assertEqual(out.count(r"\psframe"), 3)
        self.assertIn(r"\psframe(2.95,-0.05)(3.05,0.05)", out)

    def test_labels_corners_and_blocks(self):
        out = pst("A: (1,1)\nB: box at A\nline from B.n to B.s\n[ S: Here; line right 1 ] with .S at (2,0)")
        self.assertIn(r"\psline(1,1.25)(1,0.75)", out)
        self.assertIn(r"\psline(2,0)(3,0)", out)

    def test_if_else_selects_one_branch(self):
        out = pst('x = 2\nif x > 1 then { "big" at (0,0) } else { "small" at (0,0) }')
        self.assertIn("{big}", out)
        self.assertNotIn("small", out)

    def test_arc_placed_by_its_centre(self):
        out = pst("arc cw from (0,0) to (1,1) with .c at (1,0)")
        match = re.search(r"\\psarcn\[linewidth=0\.8pt\]\(1,0\)\{1\}\{(-?[\d.]+)\}\{(-?[\d.]+)\}", out)
        self.assertIsNotNone(match)
        start, end = (float(v) % 360 for v in match.groups())
        self.assertAlmostEqual(start, 180)
        self.assertAlmostEqual(end, 90)

    def test_scale_and_text_justification(self):
        out = pst('scale = 25.4\nline from (0,0) to (25.4,0) "x" above')
        self.assertIn(r"\psline(0,0)(1,0)", out)
        self.assertIn(r"\uput{2bp}[u](0.5,0){x}", out)

    def test_accepts_crlf_and_preserves_lines_outside_the_picture(self):
        out = compile_pic("\\psset{arrowsize=1pt}%\r\n.PS\r\nline from (0,0) to (1,0)\r\n.PE\r\n")
        self.assertTrue(out.startswith("\\psset{arrowsize=1pt}%\n"))
        self.assertIn(r"\psline(0,0)(1,0)", out)

    def test_macro_arguments_are_substituted_lazily_like_dpic(self):
        out = pst('define pick { exec sprintf("$%g", $1+1); }\n'
                  'define mk { pick(2, "a", "$1-b") }\n'
                  'mk(z)\n'
                  'define name { define $1 {"[$2]"} }\n'
                  'name(Col, ignored)\n'
                  'Col at (1,1)')
        self.assertIn("{z-b}", out)
        self.assertNotIn("{a}", out)
        self.assertIn(r"\rput(1,1){[]}", out)

    def test_scaled_location_coordinate(self):
        out = pst("A: (1,1.2)\nx = A*0.5.y^2\nline from (0,0) to (x,0)")
        self.assertIn(r"\psline(0,0)(0.36,0)", out)

    def test_reports_errors_but_keeps_going(self):
        compiler = PicCompiler()
        out = compiler.compile(".PS\nline from Nowhere to (1,0)\nbox at (0,0)\n.PE\n")
        self.assertTrue(compiler.errors)
        self.assertIn(r"\psframe(-0.375,-0.25)(0.375,0.25)", out)


class CommandLineTests(unittest.TestCase):
    def test_compiles_a_file_and_copies_from_an_include_folder(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "lib").mkdir()
            (root / "lib" / "shapes.pic").write_text("define sq { box wid $1 ht $1 }\n", encoding="utf-8")
            (root / "fig.pic").write_text('.PS\ncopy "shapes.pic"\nsq(0.5)\n.PE\n', encoding="utf-8")
            output = root / "fig.tex"
            for dpic_option in ([], ["-p"]):
                code = main([*dpic_option, "-I", str(root / "lib"), "-o", str(output), str(root / "fig.pic")])
                text = output.read_text(encoding="utf-8")
                self.assertEqual(code, 0)
                self.assertIn("% pppic (dpic 2025.08.01 compatible)", text)
                self.assertIn(r"\psframe(0,-0.25)(0.5,0.25)", text)


class ExampleTests(unittest.TestCase):
    def test_every_example_compiles_without_errors(self):
        examples = sorted((Path(__file__).resolve().parent.parent / "examples").glob("*.pic"))
        if not examples:
            self.skipTest("examples folder not available")
        for path in examples:
            with self.subTest(example=path.name):
                compiler = PicCompiler(base_dir=path.parent)
                out = compiler.compile(path.read_text(encoding="utf-8"))
                self.assertEqual(compiler.errors, [])
                self.assertIn(r"\begin{pspicture}", out)


if __name__ == "__main__":
    unittest.main()