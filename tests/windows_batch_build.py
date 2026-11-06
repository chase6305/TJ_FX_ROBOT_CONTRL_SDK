"""Exercise the real cmd.exe batch flow with an inert compiler replacement."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    if os.name != "nt":
        raise SystemExit("This regression test requires Windows cmd.exe")

    root = Path(__file__).resolve().parents[1]
    source = root / "c++_win"
    expected = {path.name for path in source.glob("*.cpp")}
    assert len(expected) == 18, expected
    framework = Path(os.environ["WINDIR"]) / "Microsoft.NET"
    candidates = [framework / kind / "v4.0.30319/csc.exe"
                  for kind in ("Framework64", "Framework")]
    compiler = next((path for path in candidates if path.is_file()), None)
    if compiler is None:
        raise RuntimeError(".NET Framework C# compiler was not found")

    with tempfile.TemporaryDirectory(prefix="sdk batch test ") as temporary:
        temporary = Path(temporary)
        sdk = temporary / "sdk folder"
        fake_bin = temporary / "fake compiler"
        caller = temporary / "caller folder"
        for directory in (sdk, fake_bin, caller):
            directory.mkdir()
        batch = sdk / "auto_complie.bat"
        shutil.copyfile(source / batch.name, batch)
        fake_source = temporary / "FakeCompiler.cs"
        fake_source.write_text('''
using System;
using System.IO;
class FakeCompiler {
    static int Main(string[] args) {
        if (args.Length == 0) return 2;
        File.AppendAllText(Environment.GetEnvironmentVariable("SDK_BATCH_TEST_LOG"),
            Directory.GetCurrentDirectory() + "|" + args[0] + Environment.NewLine);
        return args[0] == Environment.GetEnvironmentVariable("SDK_BATCH_TEST_FAIL") ? 1 : 0;
    }
}
''', encoding="utf-8")
        subprocess.run([str(compiler), "/nologo", "/target:exe",
                        "/out:" + str(fake_bin / "g++.exe"), str(fake_source)],
                       check=True, timeout=60)

        for failed_source in ("", "my_dd.cpp", "pvt_demo.cpp",
                              "drag_CartImdedance_save_data_arm_A.cpp"):
            log = temporary / "compiler.log"
            if log.exists():
                log.unlink()
            environment = os.environ.copy()
            environment.update({
                "PATH": str(fake_bin) + os.pathsep + environment["PATH"],
                "SDK_BATCH_TEST_LOG": str(log),
                "SDK_BATCH_TEST_FAIL": failed_source,
                "SDK_BUILD_FAILED": "99",
            })
            result = subprocess.run(
                [environment["COMSPEC"], "/d", "/c", "call", str(batch)],
                cwd=caller, env=environment, input=b"", capture_output=True,
                timeout=60)
            output = (result.stdout + result.stderr).decode(errors="replace")
            if bool(result.returncode) != bool(failed_source):
                raise AssertionError((failed_source, result.returncode, output))
            records = [line.split("|", 1) for line in
                       log.read_text(encoding="utf-8-sig").splitlines()]
            assert len(records) == len(expected), (records, output)
            assert {name for _, name in records} == expected, (records, output)
            assert all(Path(cwd) == sdk for cwd, _ in records), records
            print("PASS:", failed_source or "all compilers succeed")


if __name__ == "__main__":
    main()
