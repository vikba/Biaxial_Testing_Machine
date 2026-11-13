"""Bundle the shared config template without duplicating it in the source tree."""
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py


class BuildPy(build_py):
    def run(self):
        super().run()
        source = Path(__file__).resolve().parent / "resources" / "config.example.json"
        target = Path(self.build_lib) / "biaxial_testing_machine" / "resources"
        self.mkpath(str(target))
        self.copy_file(str(source), str(target / source.name))


setup(cmdclass={"build_py": BuildPy})
