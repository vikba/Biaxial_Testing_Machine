from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
from math import isfinite
from typing import Any

from .unit import Unit


AXIS_TRAVEL_MM = 83.0


@dataclass
class ForceConfig:
    axis1_serial: str = "0017835720"
    axis2_serial: str = "0017835697"
    # Preserve the old /2 correction until mechanical geometry is verified.
    axis1_transmission_factor: float = 0.5
    axis2_transmission_factor: float = 0.5
    timeout: float = 0.075
    max_data_age: float = 0.25

    def __post_init__(self):
        serials = (self.axis1_serial, self.axis2_serial)
        if any(not isinstance(s, str) or len(s) != 10 or not s.isdecimal() for s in serials):
            raise ValueError("DSCUSB serials must be ten-digit strings")
        if serials[0] == serials[1]:
            raise ValueError("Each axis requires a different DSCUSB serial")
        for value in (self.axis1_transmission_factor, self.axis2_transmission_factor,
                      self.timeout, self.max_data_age):
            if not isfinite(value) or value <= 0:
                raise ValueError("Force factors and timeouts must be finite and positive")
        if 2 * self.timeout >= self.max_data_age:
            raise ValueError("Maximum force age must exceed two serial timeouts")


@dataclass
class RuntimeConfig:
    unit: str = "newton"
    folder: str = ""
    sample_name: str = ""
    ip_motor: str = "172.31.100.108"
    camera_id: str = ""


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
    force: ForceConfig = field(default_factory=ForceConfig)

    def validate(self, tab_index: int | None = None) -> None:
        """Validate user input before changing session state or issuing commands."""
        if self.runtime.unit not in ("gram", "newton"):
            raise ValueError("Unit must be gram or newton.")
        sections = [(self.sample, tuple(vars(self.sample)))]
        fields = ("tareload1", "tareload2")
        if tab_index == 0:
            fields += ("load_test1", "load_test2", "disp_guess1", "disp_guess2", "halfcycltime")
        sections.append((self.load_test, fields))
        if tab_index == 2:
            sections.append((self.displacement_test, ("disptestspeed1", "disptestspeed2", "disptestlen1", "disptestlen2")))
        elif tab_index not in (None, 0):
            raise ValueError("The selected tab is not supported.")
        for section, names in sections:
            for name in names:
                try:
                    value = float(getattr(section, name))
                except (TypeError, ValueError, OverflowError):
                    raise ValueError(f"{name} must be a number.") from None
                if not isfinite(value) or value < 0 or (value == 0 and not name.startswith("tareload")):
                    raise ValueError(f"{name} must be finite and {'nonnegative' if name.startswith('tareload') else 'positive'}.")
                # Each axis supplies half the configured displacement; travel is 83 mm.
                if name.startswith(("disp_guess", "disptestlen")) and value > 2 * AXIS_TRAVEL_MM:
                    raise ValueError(f"{name} exceeds the machine's {AXIS_TRAVEL_MM:g} mm per-axis travel.")
        if tab_index is not None:
            section = self.load_test if tab_index == 0 else self.displacement_test
            names = ("cycleprecond", "cycletest") if tab_index == 0 else ("disptestcycle",)
            for name in names:
                try:
                    value = int(str(getattr(section, name)))
                except (TypeError, ValueError, OverflowError):
                    raise ValueError(f"{name} must be a whole number.") from None
                if value < 0 or (tab_index == 2 and value == 0):
                    raise ValueError(f"{name} has an invalid cycle count.")
            if tab_index == 0:
                if int(section.cycleprecond) + int(section.cycletest) == 0:
                    raise ValueError("At least one test or preconditioning cycle is required.")
                if any(float(getattr(section, f"tareload{axis}")) >= float(getattr(section, f"load_test{axis}")) for axis in (1, 2)):
                    raise ValueError("Target loads must exceed tare loads.")

    @property
    def unit_enum(self) -> Unit:
        return Unit.Gram if self.runtime.unit == "gram" else Unit.Newton

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)



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
            camera_id=str(runtime_data.get("camera_id", data.get("camera_id", ""))).strip(),
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
            force=ForceConfig(**data.get("force", {})),
            runtime=runtime,
            sample=sample,
            load_test=load_test,
            displacement_test=displacement_test,
        )

    def save(self, config: MachineConfig, path: str | None = None) -> None:
        target = path or self._path
        with open(target, "w", encoding="utf-8") as file:
            json.dump(config.to_dict(), file, indent=4)
