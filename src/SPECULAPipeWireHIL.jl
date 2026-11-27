module SPECULAPipeWireHIL

using AdaptiveOpticsSim.AlgorithmGraphs:
    AlgorithmGraphs, adopt_hil_command!, hil_command_buffer, hil_frame_buffer,
    hil_boundary_status, step_hil_frame!, reset_hil_boundary!
using AdaptiveOpticsSimPipeWireHIL:
    PipeWireHILConfiguration, PreparedPipeWireHIL, SimRunner, exchange_frame!
import AdaptiveOpticsSimPipeWireHIL: prepare_pipewire_hil, reset_pipewire_hil!
using PipeWireAO: SPA, Pod, header_metadata_param, metadata_param, roundtrip,
    update_params!, with_thread_loop_lock
using PythonCall: Py, PyArray, pybuiltins, pyconvert, pydict, pyis

export PipeWireHILConfiguration, PreparedPipeWireHIL, prepare_pipewire_hil,
    exchange_frame!, reset_pipewire_hil!
public Boundary, CameraSession, acquisition_mode, prepare, prepare_boundary,
    with_cameras, step!, receive_commands!, wait_for_streams!

include("boundary.jl")
include("cameras.jl")

end
