# Majd
"""Keep local documentation links and source dependencies consistent."""

from __future__ import annotations

import ast
import importlib.util
import re
import unittest
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


class Imports(ast.NodeVisitor):
    def __init__(self):
        self.targets = []

    def visit_If(self, node):
        if isinstance(node.test, ast.Name) and node.test.id == "TYPE_CHECKING":
            for child in node.orelse:
                self.visit(child)
        else:
            self.generic_visit(node)

    def visit_Import(self, node):
        self.targets.extend(alias.name for alias in node.names)

    def visit_ImportFrom(self, node):
        self.targets.append("." * node.level + (node.module or ""))


class ProjectStructureTests(unittest.TestCase):
    def test_documentation_targets_exist_with_exact_filename_case(self):
        documents = [ROOT / "README.md"]
        for directory in (
            "src",
            "docs",
            "scripts",
            "research",
            "data",
            "hardware",
            "firmware",
            "examples",
            "tests",
            "artifacts/models",
        ):
            documents.extend((ROOT / directory).rglob("*.md"))
        for path in documents:
            if any(
                part.startswith(".") or part.endswith(".egg-info")
                for part in path.relative_to(ROOT).parts
            ):
                continue
            for target in re.findall(
                r"\]\((<[^>]+>|[^\s)]+)\)", path.read_text(encoding="utf-8-sig")
            ):
                parsed = urlsplit(target.strip("<>"))
                if parsed.scheme or parsed.netloc or not parsed.path:
                    continue
                resolved = (path.parent / unquote(parsed.path)).resolve()
                with self.subTest(document=path.relative_to(ROOT), target=target):
                    self.assertTrue(resolved.exists(), str(resolved))
                    while resolved != ROOT and resolved != resolved.parent:
                        self.assertIn(resolved.name, [p.name for p in resolved.parent.iterdir()])
                        resolved = resolved.parent

    def test_internal_dependencies_resolve_without_runtime_cycles(self):
        modules = {}
        for path in (ROOT / "src/aurora").rglob("*.py"):
            name = ".".join(path.relative_to(ROOT / "src").with_suffix("").parts)
            modules[name.removesuffix(".__init__")] = path
        edges = {}
        for name, path in modules.items():
            visitor = Imports()
            visitor.visit(ast.parse(path.read_text(encoding="utf-8-sig")))
            package = name if path.name == "__init__.py" else name.rsplit(".", 1)[0]
            targets = [
                importlib.util.resolve_name(target, package) if target.startswith(".") else target
                for target in visitor.targets
            ]
            edges[name] = {target for target in targets if target.startswith("aurora.")}
            for target in edges[name]:
                self.assertIn(target, modules, f"{name} imports missing module {target}")
        done = set()

        def visit(name, parents):
            self.assertNotIn(name, parents, " -> ".join((*parents, name)))
            if name in done:
                return
            for target in edges[name]:
                visit(target, (*parents, name))
            done.add(name)

        for name in modules:
            visit(name, ())


if __name__ == "__main__":
    unittest.main()
