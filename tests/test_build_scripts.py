"""Test shell control flow with inert compiler replacements, never SDK programs."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = [('marvinSDK_ubuntu.sh', 'contrlSDK', False),
           ('marvinSDK_ubuntu_100343.sh', 'contrlSDK100343', True)]


@unittest.skipUnless(os.name == 'posix', 'Linux build scripts require POSIX tools')
class BuildScriptTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='sdk build test ')
        self.addCleanup(self.directory.cleanup)
        self.base = Path(self.directory.name)
        self.root = self.base / 'sdk folder'
        self.caller = self.base / 'caller folder'
        self.bin = self.base / 'fake tools'
        for path in [self.root, self.caller, self.bin]:
            path.mkdir()
        for folder in ['contrlSDK', 'contrlSDK100343', 'kinematicsSDK',
                       'interferenceCheck/Interf', 'interferenceCheck/FxCfg',
                       'SDK_PYTHON', 'DEMO_C++']:
            (self.root / folder).mkdir(parents=True)
        for name, _, _ in SCRIPTS:
            shutil.copyfile(ROOT / name, self.root / name)
        for folder in ['SDK_PYTHON', 'DEMO_C++']:
            for name in ['libMarvinSDK.so', 'libKine.so', 'libInterfCheck.so']:
                (self.root / folder / name).write_bytes(b'installed-old')
        (self.root / 'interferenceCheck/Interf/stub.cpp').touch()
        (self.root / 'interferenceCheck/FxCfg/CfgBase.cpp').touch()
        make = self.bin / 'make'
        make.write_text('''#!/usr/bin/env python3
import os,sys
from pathlib import Path
args=sys.argv[1:]
if 'clean' in args: sys.exit(0)
folder=Path.cwd()
if '-C' in args: folder=folder/args[args.index('-C')+1]
if folder.name not in ['contrlSDK','contrlSDK100343','kinematicsSDK']: sys.exit(2)
if os.environ.get('SDK_TEST_FAIL')==folder.name: sys.exit(7)
name='libKine.so' if folder.name=='kinematicsSDK' else 'libMarvinSDK.so'
(folder/name).write_bytes(('built-'+folder.name).encode())
''', encoding='utf-8')
        compiler = self.bin / 'g++'
        compiler.write_text('''#!/usr/bin/env python3
import os,sys
from pathlib import Path
if os.environ.get('SDK_TEST_FAIL')=='interference': sys.exit(9)
Path(sys.argv[sys.argv.index('-o')+1]).write_bytes(b'built-interference')
''', encoding='utf-8')
        for tool in [make, compiler]:
            tool.chmod(0o755)

    def execute(self, script, failure='', from_caller=False):
        env = dict(os.environ, PATH=str(self.bin) + os.pathsep + os.environ['PATH'],
                   SDK_TEST_FAIL=failure)
        return subprocess.run(['bash', str(self.root / script)],
                              cwd=self.caller if from_caller else self.root,
                              env=env, capture_output=True, text=True, timeout=20)

    def assert_installed_old(self):
        for folder in ['SDK_PYTHON', 'DEMO_C++']:
            for name in ['libMarvinSDK.so', 'libKine.so', 'libInterfCheck.so']:
                self.assertEqual((self.root / folder / name).read_bytes(), b'installed-old')

    def test_success_from_other_directory_with_spaces(self):
        for script, controller, interference in SCRIPTS:
            with self.subTest(script=script):
                result = self.execute(script, from_caller=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                for folder in ['SDK_PYTHON', 'DEMO_C++']:
                    expected = {'libMarvinSDK.so': ('built-' + controller).encode(),
                                'libKine.so': b'built-kinematicsSDK'}
                    if interference:
                        expected['libInterfCheck.so'] = b'built-interference'
                    for name, content in expected.items():
                        path = self.root / folder / name
                        self.assertEqual(path.read_bytes(), content)
                        self.assertEqual(path.stat().st_mode & 0o777, 0o755)

    def test_each_failed_build_preserves_installed_libraries(self):
        for script, controller, interference in SCRIPTS:
            failures = [controller, 'kinematicsSDK'] + (['interference'] if interference else [])
            for failure in failures:
                with self.subTest(script=script, failure=failure):
                    result = self.execute(script, failure=failure)
                    self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assert_installed_old()

    def test_build_only_without_consumer_directories(self):
        shutil.rmtree(self.root / 'SDK_PYTHON')
        shutil.rmtree(self.root / 'DEMO_C++')
        for script, _, _ in SCRIPTS:
            with self.subTest(script=script):
                result = self.execute(script)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


@unittest.skipUnless(os.name == 'posix' and shutil.which('make'), 'GNU make required')
class MakeDependenciesTests(unittest.TestCase):
    def test_secondary_sources_and_headers_invalidate_library(self):
        for module, primary, secondary, library in [
                ('contrlSDK', 'MarvinSDK.cpp', 'Robot', 'libMarvinSDK.so'),
                ('contrlSDK100343', 'MarvinSDK.cpp', 'Robot', 'libMarvinSDK.so'),
                ('kinematicsSDK', 'FxRobot.cpp', 'AxisPln', 'libKine.so')]:
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                shutil.copyfile(ROOT / module / 'makefile', directory / 'makefile')
                for name in [primary, secondary + '.cpp', secondary + '.h']:
                    path = directory / name
                    path.touch()
                    os.utime(path, (100, 100))
                output = directory / library
                output.touch()
                os.utime(output, (200, 200))
                def status():
                    return subprocess.run(['make', '-q', library], cwd=directory,
                                          capture_output=True, timeout=10).returncode
                self.assertEqual(status(), 0)
                for name in [secondary + '.cpp', secondary + '.h']:
                    with self.subTest(module=module, changed=name):
                        path = directory / name
                        os.utime(path, (300, 300))
                        self.assertEqual(status(), 1, 'Changed source/header must trigger recompilation')
                        os.utime(path, (100, 100))


if __name__ == '__main__':
    unittest.main()
