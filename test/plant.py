"""Small deterministic Python plant for adapter contracts and native transport tests."""

from types import SimpleNamespace

import numpy as np


class Plant:
    def __init__(self, adc=True, cadence=2_000_000):
        self.period_ns = cadence
        self.index = 0
        self.fault = None
        self.reference_open = None
        self.skip = False
        self.command_counts = {"primary": 2, "secondary": 1}
        self.cameras = {
            name: SimpleNamespace(exposure_ns=cadence, adc={"padding_rows": [1]} if adc else None,
                                  node=SimpleNamespace(outputs={"out_pixels":
                                                                SimpleNamespace(pixels=np.zeros((2, 3)))}))
            for name in ("wfs", "science")
        }
        self.last_commands = {}

    def step(self, *, commands, pupil_opd_m):
        self.last_commands = {name: np.array(command) for name, command in commands.items()}
        level = 20 + sum(command.sum() for command in self.last_commands.values())
        self.index += 1
        if self.skip:
            return {}
        if self.fault == "raise":
            raise ValueError("plant step failed")
        results = {}
        for name, camera in self.cameras.items():
            pixels = np.full((3, 3) if camera.adc else (2, 3), level,
                             dtype=np.uint16 if camera.adc else np.float32)
            if self.fault == "shape":
                pixels = pixels[:-1]
            if self.fault == "dtype":
                pixels = pixels.astype(np.float64)
            results[name] = SimpleNamespace(pixels=pixels, end_ns=self.index * self.period_ns,
                                            reference_open=self.reference_open)
        return results
