# SPECULAPipeWireHIL.jl

[![CI and Aqua](https://github.com/DarrylGamroth/SPECULAPipeWireHIL.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/DarrylGamroth/SPECULAPipeWireHIL.jl/actions/workflows/ci.yml)
[![Codecov](https://codecov.io/gh/DarrylGamroth/SPECULAPipeWireHIL.jl/graph/badge.svg)](https://codecov.io/gh/DarrylGamroth/SPECULAPipeWireHIL.jl)

Connect a prepared [SPECULA](https://github.com/ArcetriAdaptiveOptics/SPECULA)
plant to PipeWireAO. The adapter uses the buffered plant interface and
transport of [AdaptiveOpticsSimPipeWireHIL](https://github.com/DarrylGamroth/AdaptiveOpticsSimPipeWireHIL).
SPECULA owns the optics and detector model. The RTC owns deployment, session
control, and controller graphs. The application supplies the plant and port
declarations.

## Install

Use Julia 1.13 or newer on Linux and configure PythonCall with an environment
that includes SPECULA. For the adapter checkout, instantiate with the public
Julia registry. Its project pins the public HIL source revision.

```sh
git clone https://github.com/DarrylGamroth/SPECULAPipeWireHIL.jl.git
cd SPECULAPipeWireHIL.jl
JULIA_PKG_USE_CLI_GIT=true julia --startup-file=no --project=. -e '
    using Pkg
    Pkg.Registry.add("General")
    Pkg.Registry.add(url="https://github.com/DarrylGamroth/PackageRegistry.git")
    Pkg.instantiate()'
```

For a Python virtual environment owned by the caller, set
`JULIA_PYTHONCALL_EXE` to its Python executable and
`JULIA_CONDAPKG_BACKEND=Null` before loading Julia.

## Use the same HIL exchange

```julia
using SPECULAPipeWireHIL
using PipeWireAO: start!

boundary, configuration = SPECULAPipeWireHIL.prepare_boundary(plant, wire, socket)
prepared = prepare_pipewire_hil(boundary, configuration)
try
    # Connect the camera and DM ports under the session owner's control.
    start!(prepared)
    exchange_frame!(prepared)
finally
    close(prepared)
end
```

The input `plant` is a prepared Python object. It exposes `period_ns`,
`command_counts`, and `cameras`, and advances with
`step(commands=..., pupil_opd_m=...)`. Each returned exposure provides
the detector `pixels`, model `end_ns`, and optional `reference_open` value.
The wire declaration supplies camera and mirror names, node names, row-major
array shapes, frame and command schemas, command scale, frame encoding,
sample rate, and exposure duration. Shapes, cadence, and command extent are
checked before opening ports. The [contract](docs/adapter.md) describes
the plant properties and wire keys, including multiple command ports.

Lockstep returns the same `PreparedPipeWireHIL` type and uses the same
`exchange_frame!` method and `reset_pipewire_hil!` operation as the AOS
transport. Pass a fresh Python plant to reset optical model time with the
transport generation.
One frame remains outstanding until the matching command set is adopted.
Free-running acquisition uses `SPECULAPipeWireHIL.with_cameras` and
`step!` for independently clocked exposures and supports chopped-camera
metadata, camera regions, and named DM ports.

## Verification

The package tests exercise the Python boundary, Aqua, and native streams on
a disposable PipeWireAO core. CI enforces line coverage above 90% and uses
OIDC to upload the report to Codecov. Application-level qualification uses a
separately prepared RTC and calibration worker.

## License and attribution

The adapter uses the [MIT license](LICENSE). The original Julia adapter is
by Darryl Gamroth. The lockstep and multi-camera transport code were extracted
from the instrument tooling at revision `66a072d`. Python plant construction
and instrument data remain caller-owned. Cite [Rossi, Puglisi, and Agapito
(2026)](https://doi.org/10.1117/1.JATIS.12.1.019001) when publishing
results obtained with SPECULA.
