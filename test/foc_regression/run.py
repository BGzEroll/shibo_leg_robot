#!/usr/bin/env python3
"""构建并执行实际定点核心及硬件替身下的任务链；不访问目标板。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
tests = root / "test/foc_regression"
drivers = root / "src/drivers"
common = ["g++", "-std=c++17", "-O1", "-g", "-Wall", "-Wextra",
          "-Wno-unused-parameter", "-fsanitize=undefined", "-fno-sanitize-recover=all",
          "-I" + str(tests / "stubs"), "-I" + str(drivers),
          "-I" + str(root / "src/devices")]
with tempfile.TemporaryDirectory(prefix="shibo-foc-") as temp:
    for name, sources in {
        "core": [tests / "core.cpp", drivers / "foc_motor.cpp"],
        "pipeline": [tests / "pipeline.cpp", drivers / "foc_motor.cpp",
                     drivers / "bus/i2c_bus.cpp", drivers / "mpu6050.cpp",
                     root / "src/devices/hw/sensor.cpp"],
    }.items():
        binary = str(Path(temp) / name)
        subprocess.run(common + [str(p) for p in sources] + ["-o", binary], check=True)
        subprocess.run([binary], check=True)
