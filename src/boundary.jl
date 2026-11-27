# Only the exchange owner calls Python. The HIL transport's native callbacks
# handle the prepared Julia buffers and validate same-exposure metadata.
"""Prepared SPECULA plant and host buffers for the shared lockstep exchange."""
mutable struct Boundary{T,P,M,C}
    plant::Py
    camera::String
    mirror::M
    pupil_opd_m::P
    frame::Matrix{T}
    command::C
    active_command::C
    sequence::UInt64
    active_command_sequence::UInt64
    command_response_required::Bool
    failed::Bool
end

AlgorithmGraphs.hil_frame_buffer(boundary::Boundary) = boundary.frame
AlgorithmGraphs.hil_command_buffer(boundary::Boundary) = boundary.command

clear_commands!(command::AbstractVector) = fill!(command, 0)
clear_commands!(commands::NamedTuple) = foreach(clear_commands!, values(commands))

function AlgorithmGraphs.reset_hil_boundary!(boundary::Boundary)
    fill!(boundary.frame, 0)
    clear_commands!(boundary.command)
    clear_commands!(boundary.active_command)
    boundary.sequence = boundary.active_command_sequence = 0
    boundary.command_response_required = boundary.failed = false
    return nothing
end

function AlgorithmGraphs.hil_boundary_status(boundary::Boundary)
    return (
        frame_sequence=boundary.sequence,
        active_command_sequence=boundary.active_command_sequence,
        command_response_required=boundary.command_response_required,
        failed=boundary.failed,
    )
end

command_map(mirror::String, command) = pydict(Dict(mirror => command))
command_map(::Tuple, command::NamedTuple) = pydict(Dict(String(key) => value for (key, value) in pairs(command)))

function adopt!(destination::Vector{Float32}, command::Vector{Float32})
    all(isfinite, command) || error("DM command contains nonfinite values")
    copyto!(destination, command)
end

function adopt!(destination::NamedTuple, command::NamedTuple)
    all(values -> all(isfinite, values), command) || error("DM command contains nonfinite values")
    foreach(copyto!, values(destination), values(command))
end

function AlgorithmGraphs.step_hil_frame!(boundary::Boundary)
    boundary.failed && error("SPECULA exchange failed")
    boundary.command_response_required && error("Frame is still awaiting its DM command")
    try
        commands = command_map(boundary.mirror, boundary.active_command)
        result = boundary.plant.step(; commands, pupil_opd_m=boundary.pupil_opd_m)
        exposure = result[boundary.camera]
        pixels = PyArray(exposure.pixels)
        axes(pixels) == axes(boundary.frame) || error("SPECULA camera shape changed")
        eltype(pixels) == eltype(boundary.frame) || error("SPECULA camera element type changed")
        copyto!(boundary.frame, pixels)
        boundary.sequence += 1
        boundary.command_response_required = true
        return boundary.sequence
    catch
        boundary.failed = true
        rethrow()
    end
end

function AlgorithmGraphs.adopt_hil_command!(boundary::Boundary, sequence::UInt64)
    boundary.failed && error("SPECULA exchange failed")
    boundary.command_response_required && sequence == boundary.sequence ||
        error("DM command does not match the outstanding exposure")
    adopt!(boundary.active_command, boundary.command)
    boundary.active_command_sequence = sequence
    boundary.command_response_required = false
    return sequence
end

function configuration(wire, remote; node=wire["command_node_name"], schema=wire["command_schema"],
        scale=wire["command_scale"])
    isabspath(remote) || throw(ArgumentError("Use the absolute path of the private PipeWire socket"))
    return PipeWireHILConfiguration(;
        remote, rate=SPA.Fraction(wire["rate"]...),
        frame_node_name=wire["frame_node_name"],
        command_node_name=node,
        frame_schema=wire["frame_schema"],
        command_schema=schema,
        frame_encoding=Symbol(wire["frame_encoding"]),
        command_scale=scale,
        exposure_duration_ns=wire["exposure_duration_ns"],
        timeout_ns=wire["timeout_ns"],
    )
end

function camera_format(detector)
    shape = pyconvert(Tuple{Int,Int}, detector.node.outputs["out_pixels"].pixels.shape)
    adc = detector.adc
    pyis(adc, pybuiltins.None) && return shape, Float32
    return (shape[1] + sum(pyconvert(Vector{Int}, adc["padding_rows"])), shape[2]), UInt16
end

function command_buffer(plant, mirror, count)
    pyconvert(Int, plant.command_counts[mirror]) == count ||
        throw(ArgumentError("Wire extent differs from SPECULA for $mirror"))
    return zeros(Float32, count)
end

function command_configuration(wire, remote, plant)
    if !haskey(wire, "commands")
        mirror = wire["mirror"]
        return configuration(wire, remote), mirror, command_buffer(plant, mirror, wire["command_count"])
    end
    commands = wire["commands"]
    mirrors = Tuple(item["mirror"] for item in commands)
    names = Tuple(Symbol(name) for name in mirrors)
    field(key) = NamedTuple{names}(Tuple(item[key] for item in commands))
    config = configuration(wire, remote; node=field("node_name"), schema=field("schema"), scale=field("scale"))
    command = NamedTuple{names}(Tuple(command_buffer(plant, item["mirror"], item["count"]) for item in commands))
    return config, mirrors, command
end

function prepare_command_metadata!(transport)
    streams = transport.command_stream
    schemas = transport.configuration.command_schema
    inputs = schemas isa NamedTuple ?
        ((streams[key], schema) for (key, schema) in pairs(schemas)) : ((streams, schemas),)
    physical_opd = ("pipewireao/dm-command-opd-um-f32/1", "pipewireao/actuator-command-opd-m-f32/1")
    targets = [stream for (stream, schema) in inputs if schema in physical_opd]
    isempty(targets) && return nothing

    # Accept the record layouts that the RTC preserves on physical DM commands.
    records = (ACQUISITION_METADATA..., CHOPPING_METADATA...)
    params = (Pod(header_metadata_param()), map(record -> Pod(metadata_param(record)), records)...)
    with_thread_loop_lock(transport.loop) do _
        foreach(stream -> update_params!(stream, params), targets)
    end
    roundtrip(transport.core)
    return nothing
end

"""Select lockstep or free-running acquisition from the wire declaration."""
function acquisition_mode(wire, override=nothing)
    selected = something(override, get(wire, "mode", haskey(wire, "cameras") ? "free-running" : "lockstep"))
    selected in ("free-running", "lockstep") ||
        throw(ArgumentError("Select free-running or lockstep SPECULA acquisition"))
    selected == "lockstep" && haskey(wire, "cameras") &&
        throw(ArgumentError("SPECULA lockstep requires a single camera"))
    return selected
end

"""
    prepare_boundary(plant, wire, remote; pupil_opd_m=nothing)

Validate a Python SPECULA plant and return `(boundary, configuration)` for
the shared `prepare_pipewire_hil` interface. The plant provides `cameras`,
`command_counts`, `period_ns`, and `step(; commands, pupil_opd_m)`.
The optional entrance OPD is in metres. Advance the plant on its owner task.
"""
function prepare_boundary(plant::Py, wire::AbstractDict, remote; pupil_opd_m=nothing)
    config, mirror, command = command_configuration(wire, remote, plant)

    camera = wire["camera"]
    detector = plant.cameras[camera]
    period_ns = pyconvert(Int, plant.period_ns)
    exposure_ns = pyconvert(Int, detector.exposure_ns)
    Int128(period_ns) * config.rate.num == 1_000_000_000 * Int128(config.rate.denom) ||
        throw(ArgumentError("Wire rate differs from the SPECULA plant step"))
    exposure_ns <= period_ns || throw(ArgumentError("The camera must complete every plant step"))
    exposure_ns == config.exposure_duration_ns ||
        throw(ArgumentError("Wire exposure duration differs from the SPECULA camera"))
    shape, T = camera_format(detector)
    T === UInt16 && config.frame_encoding !== :uint16 && throw(ArgumentError("The camera produces UInt16 ADU"))
    shape == Tuple(wire["frame_shape"]) || throw(ArgumentError("Wire camera shape differs from SPECULA"))
    boundary = Boundary(
        plant, camera, mirror, pupil_opd_m, zeros(T, shape),
        command, deepcopy(command), UInt64(0), UInt64(0), false, false,
    )
    return boundary, config
end

"""
    prepare(plant, wire, remote; pupil_opd_m=nothing, transport_options...)

Open inactive SPECULA feedback endpoints. The returned `PreparedPipeWireHIL`
uses the shared exchange and reset methods. The caller owns links and closure.
"""
function prepare(plant::Py, wire::AbstractDict, remote; pupil_opd_m=nothing, transport_options...)
    boundary, config = prepare_boundary(plant, wire, remote; pupil_opd_m)
    return prepare_pipewire_hil(boundary, config; transport_options...)
end

function prepare_pipewire_hil(boundary::Boundary, config::PipeWireHILConfiguration; transport_options...)
    transport = invoke(prepare_pipewire_hil, Tuple{Any,PipeWireHILConfiguration},
        boundary, config; transport_options...)
    try
        prepare_command_metadata!(transport)
    catch err
        try
            close(transport)
        catch cleanup_error
            throw(CompositeException(Any[err, cleanup_error]))
        end
        rethrow()
    end
    return transport
end

"""
    reset_pipewire_hil!(prepared::PreparedPipeWireHIL{<:Boundary}, fresh_plant)

Reset the quiescent lockstep exchange and install a fresh instance of the
same Python plant. Prepare the replacement on the owning task before reset.
"""
function reset_pipewire_hil!(prepared::PreparedPipeWireHIL{<:Boundary}, fresh_plant::Py)
    reset_pipewire_hil!(prepared)
    prepared.boundary.plant = fresh_plant
    return prepared
end
