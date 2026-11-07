from pathlib import Path
import subprocess, shutil, json

root = Path(__file__).resolve().parents[2]
out = root / "Build/remote_tests"
out.mkdir(parents=True, exist_ok=True)
gcc, gxx = shutil.which("gcc"), shutil.which("g++")
if not gcc or not gxx:
    raise SystemExit("Host GCC/G++ not found in PATH")
includes = [root / "tests/remote_input/stubs", root / "User/BSP", root / "User/Device", root / "User/App", root / "User/Control"]
common = ["-Wall", "-Wextra", "-Werror", "-O0", "-g", "-ffunction-sections", "-fdata-sections"] + [arg for p in includes for arg in ("-I", str(p))]

def source(stem):
    for suffix in (".cpp", ".c"):
        p = root / (stem + suffix)
        if p.is_file():
            return p
    raise SystemExit("Source not yet available: " + stem)

sources = [source(p) for p in ["User/Device/device_dr16", "User/Device/device_motor_dm", "User/Control/control_alg_math", "User/Control/control_alg_pid", "User/Control/control_gimbal", "User/App/app_init", "User/App/app_remote", "User/App/app_robot", "User/BSP/bsp_drv_uart", "tests/remote_input/test_remote"]]
objects = []
compiler_log = []
def build(args):
    result = subprocess.run(args, text=True, capture_output=True)
    compiler_log.append(result.stdout + result.stderr)
    (out / "compiler.log").write_text("".join(compiler_log), encoding="utf-8")
    if result.returncode:
        print(result.stdout + result.stderr)
        raise SystemExit(result.returncode)

for p in sources:
    obj = out / (p.stem + ".o")
    objects.append(str(obj))
    cpp = p.suffix == ".cpp"
    legacy = p.stem in {"device_motor_dm", "control_alg_math", "control_alg_pid"}
    warnings = ["-Wno-error=switch", "-Wno-error=enum-compare", "-Wno-error=unused-parameter", "-Wno-error=sign-compare"] if legacy else []
    build([gxx if cpp else gcc, "-std=c++11" if cpp else "-std=c11", *common, *warnings, "-c", str(p), "-o", str(obj)])
exe = out / "remote_tests.exe"
build([gxx, "-Wl,--gc-sections", *objects, "-o", str(exe)])
run = subprocess.run([str(exe)], text=True, capture_output=True)
if run.stdout:
    print(run.stdout.strip())
if run.stderr:
    print(run.stderr.strip())
report = {"passed": run.returncode == 0, "output": run.stdout.strip(), "errors": run.stderr.strip(), "scope": "Host HAL mock; real DR16 + DM Normal + App + Gimbal + UART BSP; mocked HAL/CAN submission. No physical hardware test."}
(out / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
if run.returncode:
    raise SystemExit(run.returncode)
