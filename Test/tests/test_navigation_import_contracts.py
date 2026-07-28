import ast
from pathlib import Path
import unittest

import src.navigation as navigation


PROJECT_ROOT = Path("/home/ps/ScanEverything/Test")
SCRIPTS_DIR = PROJECT_ROOT / "Scripts"


class NavigationImportContractTests(unittest.TestCase):
    def test_script_package_level_navigation_imports_are_exported(self):
        failures: list[str] = []
        exported_names = set(getattr(navigation, "__all__", ()))

        for script_path in sorted(SCRIPTS_DIR.rglob("*.py")):
            tree = ast.parse(
                script_path.read_text(encoding="utf-8"),
                filename=str(script_path),
            )

            for node in ast.walk(tree):
                if not isinstance(node, ast.ImportFrom):
                    continue
                if node.module != "src.navigation":
                    continue

                for alias in node.names:
                    if alias.name == "*":
                        failures.append(
                            f"{script_path}: wildcard import is not allowed"
                        )
                        continue

                    if not hasattr(navigation, alias.name):
                        failures.append(
                            f"{script_path}: missing src.navigation export "
                            f"{alias.name!r}"
                        )

                    if alias.name not in exported_names:
                        failures.append(
                            f"{script_path}: {alias.name!r} is imported from "
                            "src.navigation but is missing from __all__"
                        )

        if failures:
            self.fail("\n".join(failures))


if __name__ == "__main__":
    unittest.main()
