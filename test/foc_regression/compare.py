#!/usr/bin/env python3
"""编译同一测试驱动，逐行比较原库与裁剪库的实际浮点输出。"""
import argparse
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("reference", type=Path, nargs="?", help="裁剪前 foc_motor 目录")
parser.add_argument("--reference-rev", help="从本仓库指定提交提取裁剪前的库")
args = parser.parse_args()
if bool(args.reference) == bool(args.reference_rev):
    parser.error("请指定一个参考目录或 --reference-rev")
here = Path(__file__).resolve().parent
current = here.parents[1] / "lib/foc_motor"

with tempfile.TemporaryDirectory(prefix="foc_compare_") as temp:
    reference = args.reference
    if args.reference_rev:
        reference = Path(temp) / "reference_source"
        prefix = "lib/foc_motor/"
        paths = subprocess.check_output(
            ["git", "ls-tree", "-r", "--name-only", args.reference_rev, "--", prefix],
            cwd=current.parents[1], text=True).splitlines()
        if not paths:
            raise SystemExit("参考提交不含 foc_motor 库")
        for path in paths:
            destination = reference / path.removeprefix(prefix)
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(subprocess.check_output(
                ["git", "show", f"{args.reference_rev}:{path}"], cwd=current.parents[1]))
    outputs = []
    hardware_outputs = []
    for name, root in [("reference", reference.resolve()), ("trimmed", current)]:
        source = root / "src"
        files = [source / "BLDCMotor.cpp", source / "drivers/BLDCDriver3PWM.cpp"]
        files += sorted((source / "common").glob("*.cpp"))
        files += sorted((source / "common/base_classes").glob("*.cpp"))
        files += sorted((source / "communication").glob("*.cpp"))
        files += sorted((source / "sensors").glob("*.cpp"))
        executable = Path(temp) / name
        command = ["g++", "-std=c++17", "-O2", "-fno-strict-aliasing",
                   "-I", str(here / "stubs"), "-I", str(source)]
        if name == "trimmed":
            command += ["-DFOC_TRIMMED"]
        subprocess.run(command + [str(here / "regression.cpp")]
                       + [str(file) for file in files] + ["-o", str(executable)], check=True)
        outputs.append(subprocess.check_output([str(executable)]).splitlines())
        hardware_executable = Path(temp) / f"{name}_hardware"
        subprocess.run(command + ["-DESP_H", "-DARDUINO_ARCH_ESP32", "-DSOC_MCPWM_SUPPORTED",
            str(here / "hardware.cpp"), str(source / "drivers/hardware_specific/esp32_mcu.cpp"),
            str(source / "common/time_utils.cpp"), "-o", str(hardware_executable)], check=True)
        hardware_outputs.append(subprocess.check_output([str(hardware_executable)]).splitlines())
    if outputs[0] != outputs[1]:
        for index, (before, after) in enumerate(zip(*outputs)):
            if before != after:
                raise SystemExit(f"第 {index + 1} 行不同:\n原库: {before!r}\n裁剪: {after!r}")
        raise SystemExit(f"输出行数不同: {len(outputs[0])} / {len(outputs[1])}")
    print(f"PASS: {len(outputs[0])} 行输出逐字节一致（十六进制浮点）")

    if hardware_outputs[0] != hardware_outputs[1]:
        raise SystemExit("MCPWM 资源分配、计时器或 PWM 调用结果不一致")
    print(f"PASS: {len(hardware_outputs[0])} 行 MCPWM 配置与写入记录一致；零占空比初始化和资源耗尽断言通过")
