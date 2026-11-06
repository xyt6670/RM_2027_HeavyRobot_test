from pathlib import Path
import subprocess, shutil, json

root = Path(__file__).resolve().parents[2]
out = root / "Build/remote_tests"
out.mkdir(parents=True, exist_ok=True)
gcc, gxx = shutil.which("gcc"), shutil.which("g++")
if not gcc or not gxx:
    raise SystemExit("Host GCC/G++ not found in PATH")
includes = [root / "tests/remote_input/stubs", root / "User/BSP", root / "User/Device", root / "User/App"]
common = ["-Wall", "-Wextra", "-Werror", "-O0", "-g"] + [arg for p in includes for arg in ("-I", str(p))]

def source(stem):
    for suffix in (".cpp", ".c"):
        p = root / (stem + suffix)
        if p.is_file():
            return p
    raise SystemExit("Source not yet available: " + stem)

sources = [source(p) for p in ["User/Device/device_dr16", "User/App/app_init", "User/App/app_remote", "User/App/app_robot", "User/BSP/bsp_drv_uart", "tests/remote_input/test_remote"]]
objects = []
for p in sources:
    obj = out / (p.stem + ".o")
    objects.append(str(obj))
    cpp = p.suffix == ".cpp"
    subprocess.run([gxx if cpp else gcc, "-std=c++11" if cpp else "-std=c11", *common, "-c", str(p), "-o", str(obj)], check=True)
exe = out / "remote_tests.exe"
subprocess.run([gxx, *objects, "-o", str(exe)], check=True)
run = subprocess.run([str(exe)], text=True, capture_output=True)
if run.stdout:
    print(run.stdout.strip())
if run.stderr:
    print(run.stderr.strip())
report = {"passed": run.returncode == 0, "output": run.stdout.strip(), "errors": run.stderr.strip(), "scope": "Host HAL mock; real Class_DR16, app_init callback bridge, App mapping/scheduler and UART BSP. No board/receiver test."}
(out / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
if run.returncode:
    raise SystemExit(run.returncode)
