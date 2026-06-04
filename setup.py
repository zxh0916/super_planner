from __future__ import annotations

import os
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext
from setuptools.command.develop import develop


def _valid_cmake(path: str | None) -> str | None:
    if not path:
        return None
    try:
        subprocess.check_output([path, "--version"], stderr=subprocess.STDOUT)
    except (OSError, subprocess.CalledProcessError):
        return None
    return path


def _find_cmake() -> str | None:
    override = _valid_cmake(os.environ.get("SUPER_PLANNER_CMAKE"))
    if override:
        return override

    seen: set[str] = set()
    for directory in os.get_exec_path():
        candidate = Path(directory) / "cmake"
        candidate_str = str(candidate)
        if candidate_str in seen:
            continue
        seen.add(candidate_str)
        valid = _valid_cmake(candidate_str)
        if valid:
            return valid
    return None


class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "super_rosless"):
        super().__init__(name, sources=[])
        self.sourcedir = sourcedir


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        cmake = _find_cmake()
        if cmake is None:
            raise RuntimeError("cmake is required to build super_planner_py")

        repo_root = Path(__file__).resolve().parent
        source_dir = repo_root / ext.sourcedir
        build_dir = Path(os.environ.get("SUPER_PLANNER_BUILD_DIR", source_dir / "build")).resolve()
        ext_dir = Path(self.get_ext_fullpath(ext.name)).resolve().parent
        build_type = os.environ.get("CMAKE_BUILD_TYPE", "Release")
        python_include = sysconfig.get_paths().get("include")

        configure_cmd = [
            cmake,
            "-U",
            "Python3_*",
            "-U",
            "_Python3_*",
            "-U",
            "PYTHON_*",
            "-U",
            "PYBIND11_*",
            "-U",
            "pybind11_DIR",
            "-S",
            str(source_dir),
            "-B",
            str(build_dir),
            f"-DCMAKE_BUILD_TYPE={build_type}",
            f"-DPython3_EXECUTABLE={sys.executable}",
            f"-DPython3_ROOT_DIR={sys.prefix}",
            "-DPython3_FIND_VIRTUALENV=ONLY",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={ext_dir}",
        ]
        if python_include:
            configure_cmd.append(f"-DPython3_INCLUDE_DIR={python_include}")
        subprocess.check_call(configure_cmd, cwd=str(repo_root))

        build_cmd = [cmake, "--build", str(build_dir), "--config", build_type]
        parallel = os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL")
        if parallel is None:
            parallel = str(os.cpu_count() or 2)
        build_cmd.extend(["--parallel", parallel])
        subprocess.check_call(build_cmd, cwd=str(repo_root))

        built_extensions = list(ext_dir.glob(f"{ext.name}*.so"))
        if not built_extensions:
            built_extensions = list(build_dir.glob(f"{ext.name}*.so"))
        if not built_extensions:
            raise RuntimeError(f"CMake build did not produce {ext.name} extension")

        target = Path(self.get_ext_fullpath(ext.name)).resolve()
        if built_extensions[0].resolve() != target:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(built_extensions[0], target)


class DevelopWithBuild(develop):
    def run(self) -> None:
        build_ext_cmd = self.get_finalized_command("build_ext")
        build_ext_cmd.inplace = True
        self.run_command("build_ext")
        super().run()


setup(
    name="super-planner-py",
    version="0.1.0",
    description="ROS-less Python bindings for SUPER planner",
    ext_modules=[CMakeExtension("super_planner_py")],
    cmdclass={
        "build_ext": CMakeBuild,
        "develop": DevelopWithBuild,
    },
    zip_safe=False,
    install_requires=[
        "numpy",
        "pybind11",
    ],
)
