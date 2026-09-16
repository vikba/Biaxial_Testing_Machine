from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
from typing import Any

from .unit import Unit


@dataclass
class RuntimeConfig:
    unit: str = "newton"
    folder: str = ""
    sample_name: str = ""
    ip_motor: str = "172.31.100.108"
    ip_daq: str = "172.31.100.107"


@dataclass
class SampleConfig:
    thickness: float = 0.34
    length1: float = 9.0
    length2: float = 9.0
    maxstress1: float = 500.0
    maxstress2: float = 500.0

    @property
    def area1(self) -> float:
        return self.thickness * self.length1

    @property
    def area2(self) -> float:
        return self.thickness * self.length2


@dataclass
class LoadTestConfig:
    tareload1: str = "5"
    tareload2: str = "5"
    load_precond1: str = "92"
    load_precond2: str = "92"
    load_test1: str = "156.0"
    load_test2: str = "156.0"
    disp_guess1: str = "2"
    disp_guess2: str = "2"
    halfcycltime: str = "15"
    cycleprecond: str = "6"
    cycletest: str = "8"
    autoloading: bool = False


@dataclass
class DisplacementTestConfig:
    disptestspeed1: str = "12"
    disptestspeed2: str = "12"
    disptestlen1: str = "3"
    disptestlen2: str = "3"
    disptestcycle: str = "10"


@dataclass
class MachineConfig:
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    sample: SampleConfig = field(default_factory=SampleConfig)
    load_test: LoadTestConfig = field(default_factory=LoadTestConfig)
    displacement_test: DisplacementTestConfig = field(default_factory=DisplacementTestConfig)

    @property
    def unit_enum(self) -> Unit:
        return Unit.Gram if self.runtime.unit == "gram" else Unit.Newton

    def to_dict(self) -> dict[str, Any]:
        data = {
            "runtime": asdict(self.runtime),
            "sample": asdict(self.sample),
            "load_test": asdict(self.load_test),
            "displacement_test": asdict(self.displacement_test),
        }

        # Keep flat keys for compatibility with existing persisted files.
        data.update(
            {
                "name": self.runtime.sample_name,
                "unit": self.runtime.unit,
                "folder": self.runtime.folder,
                "ip_motor": self.runtime.ip_motor,
                "ip_daq": self.runtime.ip_daq,
                "thickness": self.sample.thickness,
                "length1": self.sample.length1,
                "length2": self.sample.length2,
                "maxstress1": self.sample.maxstress1,
                "maxstress2": self.sample.maxstress2,
                "tareload1": self.load_test.tareload1,
                "tareload2": self.load_test.tareload2,
                "load_precond1": self.load_test.load_precond1,
                "load_precond2": self.load_test.load_precond2,
                "load_test1": self.load_test.load_test1,
                "load_test2": self.load_test.load_test2,
                "disp_guess1": self.load_test.disp_guess1,
                "disp_guess2": self.load_test.disp_guess2,
                "halfcycltime": self.load_test.halfcycltime,
                "cycleprecond": self.load_test.cycleprecond,
                "cycletest": self.load_test.cycletest,
                "autoloading": self.load_test.autoloading,
                "disptestspeed1": self.displacement_test.disptestspeed1,
                "disptestspeed2": self.displacement_test.disptestspeed2,
                "disptestlen1": self.displacement_test.disptestlen1,
                "disptestlen2": self.displacement_test.disptestlen2,
                "disptestcycle": self.displacement_test.disptestcycle,
            }
        )
        return data


class ConfigRepository:
    def __init__(self, path: str):
        self._path = path

    @property
    def path(self) -> str:
        return self._path

    def load(self) -> MachineConfig:
        if not os.path.exists(self._path):
            return MachineConfig()

        with open(self._path, "r", encoding="utf-8") as file:
            data = json.load(file)

        runtime_data = data.get("runtime", {})
        sample_data = data.get("sample", {})
        load_data = data.get("load_test", {})
        disp_data = data.get("displacement_test", {})

        runtime = RuntimeConfig(
            unit=str(runtime_data.get("unit", data.get("unit", "newton"))).lower(),
            folder=str(runtime_data.get("folder", data.get("folder", ""))),
            sample_name=str(runtime_data.get("sample_name", data.get("name", ""))),
            ip_motor=str(runtime_data.get("ip_motor", data.get("ip_motor", RuntimeConfig.ip_motor))),
            ip_daq=str(runtime_data.get("ip_daq", data.get("ip_daq", RuntimeConfig.ip_daq))),
        )
        sample = SampleConfig(
            thickness=float(sample_data.get("thickness", data.get("thickness", SampleConfig.thickness))),
            length1=float(sample_data.get("length1", data.get("length1", SampleConfig.length1))),
            length2=float(sample_data.get("length2", data.get("length2", SampleConfig.length2))),
            maxstress1=float(sample_data.get("maxstress1", data.get("maxstress1", SampleConfig.maxstress1))),
            maxstress2=float(sample_data.get("maxstress2", data.get("maxstress2", SampleConfig.maxstress2))),
        )
        load_test = LoadTestConfig(
            tareload1=str(load_data.get("tareload1", data.get("tareload1", LoadTestConfig.tareload1))),
            tareload2=str(load_data.get("tareload2", data.get("tareload2", LoadTestConfig.tareload2))),
            load_precond1=str(load_data.get("load_precond1", data.get("load_precond1", LoadTestConfig.load_precond1))),
            load_precond2=str(load_data.get("load_precond2", data.get("load_precond2", LoadTestConfig.load_precond2))),
            load_test1=str(load_data.get("load_test1", data.get("load_test1", LoadTestConfig.load_test1))),
            load_test2=str(load_data.get("load_test2", data.get("load_test2", LoadTestConfig.load_test2))),
            disp_guess1=str(load_data.get("disp_guess1", data.get("disp_guess1", LoadTestConfig.disp_guess1))),
            disp_guess2=str(load_data.get("disp_guess2", data.get("disp_guess2", LoadTestConfig.disp_guess2))),
            halfcycltime=str(load_data.get("halfcycltime", data.get("halfcycltime", LoadTestConfig.halfcycltime))),
            cycleprecond=str(load_data.get("cycleprecond", data.get("cycleprecond", LoadTestConfig.cycleprecond))),
            cycletest=str(load_data.get("cycletest", data.get("cycletest", LoadTestConfig.cycletest))),
            autoloading=bool(load_data.get("autoloading", data.get("autoloading", LoadTestConfig.autoloading))),
        )
        displacement_test = DisplacementTestConfig(
            disptestspeed1=str(disp_data.get("disptestspeed1", data.get("disptestspeed1", DisplacementTestConfig.disptestspeed1))),
            disptestspeed2=str(disp_data.get("disptestspeed2", data.get("disptestspeed2", DisplacementTestConfig.disptestspeed2))),
            disptestlen1=str(disp_data.get("disptestlen1", data.get("disptestlen1", DisplacementTestConfig.disptestlen1))),
            disptestlen2=str(disp_data.get("disptestlen2", data.get("disptestlen2", DisplacementTestConfig.disptestlen2))),
            disptestcycle=str(disp_data.get("disptestcycle", data.get("disptestcycle", DisplacementTestConfig.disptestcycle))),
        )

        return MachineConfig(
            runtime=runtime,
            sample=sample,
            load_test=load_test,
            displacement_test=displacement_test,
        )

    def save(self, config: MachineConfig, path: str | None = None) -> None:
        target = path or self._path
        with open(target, "w", encoding="utf-8") as file:
            json.dump(config.to_dict(), file, indent=4)
