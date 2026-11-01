#!/usr/bin/env python3
"""Regression tests for the car_vel bundled runtime installer."""

import ast
import json
from pathlib import Path
import tempfile
import unittest


SETUP_PATH = Path(__file__).resolve().parents[1] / 'setup.py'


def load_install_tree():
    """Load only install_tree without executing setuptools.setup()."""
    tree = ast.parse(SETUP_PATH.read_text(encoding='utf-8'))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == 'install_tree'
    )
    namespace = {'os': __import__('os')}
    module = ast.Module(body=[function], type_ignores=[])
    exec(compile(module, str(SETUP_PATH), 'exec'), namespace)
    return namespace['install_tree']


class SetupRuntimeTreeTests(unittest.TestCase):

    def test_behavior_model_assets_keep_deployment_subdirectories(self):
        root = SETUP_PATH.parent / 'person_behavior'
        manifest = json.loads((root / 'assets/runtime_assets.json').read_text(encoding='utf-8'))
        entries = load_install_tree()(str(root), 'share/car_vel/person_behavior')
        installed = {
            Path(source).relative_to(root).as_posix():
            (Path(destination) / Path(source).name).as_posix()
            for destination, files in entries for source in files
        }
        for asset in manifest['files']:
            self.assertEqual(installed[asset['path']], 'share/car_vel/person_behavior/' + asset['path'])
        self.assertIn('pphuman_runtime/self_check.py', installed)
        self.assertIn('environment/source_provenance.json', installed)

    def test_ignores_python_cache_artifacts(self):
        install_tree = load_install_tree()
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / 'runtime'
            cache = source / '__pycache__'
            cache.mkdir(parents=True)
            (source / 'camera.py').write_text('pass\n', encoding='utf-8')
            (cache / 'camera.cpython-310.pyc').write_bytes(b'temporary')

            entries = install_tree(str(source), 'share/car_vel/runtime')
            installed = [
                Path(file_name)
                for _destination, files in entries
                for file_name in files
            ]

            self.assertIn(source / 'camera.py', installed)
            self.assertFalse(any(
                '__pycache__' in path.parts
                or path.suffix.lower() in {'.pyc', '.pyo'}
                for path in installed
            ))


if __name__ == '__main__':
    unittest.main()
