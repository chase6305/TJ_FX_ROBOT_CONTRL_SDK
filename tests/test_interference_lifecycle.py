"""Check collision-library ownership offline; no controller SDK is used."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('offline_interference', ROOT / 'SDK_PYTHON/fx_interference.py')
interference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(interference)


class InterferenceOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.wrapper = interference.Marvin_Interference.__new__(interference.Marvin_Interference)
        self.wrapper.interf = mock.Mock()
        self.wrapper._handle = None

    def test_repeated_create_keeps_owned_handle(self):
        self.wrapper.interf.FX_Interf_Create.return_value = 0x100000001
        first = self.wrapper.create()
        self.assertEqual(self.wrapper.create(), first)
        self.wrapper.interf.FX_Interf_Create.assert_called_once_with()
        self.wrapper.destroy()
        self.wrapper.destroy()
        self.wrapper.interf.FX_Interf_Destroy.assert_called_once_with(first)

    def test_create_after_destroy_allocates_again(self):
        self.wrapper.interf.FX_Interf_Create.side_effect = [11, 22]
        self.assertEqual(self.wrapper.create(), 11)
        self.wrapper.destroy()
        self.assertEqual(self.wrapper.create(), 22)
        self.assertEqual(self.wrapper.interf.FX_Interf_Create.call_count, 2)

    def test_failed_create_can_be_retried(self):
        self.wrapper.interf.FX_Interf_Create.side_effect = [None, 22]
        self.assertIsNone(self.wrapper.create())
        self.wrapper.destroy()
        self.wrapper.interf.FX_Interf_Destroy.assert_not_called()
        self.assertEqual(self.wrapper.create(), 22)


@unittest.skipUnless(sys.platform.startswith('linux') and shutil.which('g++'),
                     'Linux compiler and AddressSanitizer required')
class NativeInterferenceOwnershipTests(unittest.TestCase):
    def test_create_init_destroy_releases_allocations(self):
        with tempfile.TemporaryDirectory(prefix='interference-lifecycle-') as directory:
            directory = Path(directory)
            source = directory / 'lifecycle.cpp'
            source.write_text('''#include "Interference.h"
int main(int argc, char **argv) {
    if (argc != 6) return 1;
    FX_Interf_Destroy(nullptr);
    for (int i = 0; i < 100; ++i) {
        FX_InterfHandle handle = FX_Interf_Create();
        if (!handle) return 2;
        FX_Interf_Destroy(handle);
    }
    for (int i = 0; i < 10; ++i) {
        FX_InterfHandle handle = FX_Interf_Create();
        if (!handle) return 3;
        int result = FX_Interf_Init(handle, argv[1], argv[2], argv[3], argv[4], argv[5]);
        FX_Interf_Destroy(handle);
        if (result != FUNC_RET_SUCCESS) return 4;
    }
    return 0;
}
''', encoding='utf-8')
            interf = ROOT / 'interferenceCheck/Interf'
            executable = directory / 'lifecycle'
            command = ['g++', '-std=c++11', '-fsanitize=address', '-fno-omit-frame-pointer',
                       '-g', '-O1', '-no-pie', '-I', str(interf), str(source)]
            command += [str(path) for path in sorted(interf.glob('*.cpp'))]
            command += [str(ROOT / 'interferenceCheck/FxCfg/CfgBase.cpp'), '-o', str(executable)]
            compiled = subprocess.run(command, capture_output=True, text=True, timeout=120)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            cfg = ROOT / 'interferenceCheck/InterferenceCfg'
            definitions = ['CordDef_CCSM6Lite.Cord', 'CalLinkDef.Links',
                           'CalInputDef.Joints', 'ConvexDef_CCSM6Lite.Convex', 'InterfDef.Interf']
            env = dict(os.environ, ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',
                       LSAN_OPTIONS='exitcode=23')
            result = subprocess.run([str(executable)] + [str(cfg / name) for name in definitions],
                                    capture_output=True, text=True, env=env, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
