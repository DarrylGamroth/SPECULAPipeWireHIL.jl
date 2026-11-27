using AdaptiveOpticsSimPipeWireHIL: PipeWireArrayReceipt, PipeWireArrayStreamOptions,
    array_stream_status, drain_pipewire_array!, reset_pipewire_array!, reset_pipewire_arrays!, stage_pipewire_array!,
    with_array_stream, with_array_streams
using PipeWireAO: AcquisitionDomain, AcquisitionIdentity, MetadataRequirement, NdArray,
    NdArrayFormat, OPTICAL_FRINGED, OPTICAL_UNFRINGED, ACQUISITION_TRIGGER_HIGH,
    ACQUISITION_TRIGGER_LOW, ACQUISITION_TRIGGER_ABSENT, LibPipeWire, stream_state, trigger_process!
using UUIDs: uuid4

const ACQUISITION_METADATA = (
    MetadataRequirement(SPA.META_ACQUISITION_IDENTITY; required=true),
    MetadataRequirement(SPA.META_EXPOSURE_TIMING; required=true),
    MetadataRequirement(SPA.META_REFERENCE_TIMESTAMP; required=true),
)
const CHOPPING_METADATA = (
    MetadataRequirement(SPA.META_OPTICAL_CHOPPING),
    MetadataRequirement(SPA.META_ELECTRICAL_TRIGGER),
)
const COMMAND_METADATA = (
    MetadataRequirement(SPA.META_ACQUISITION_IDENTITY; required=true),
    MetadataRequirement(SPA.META_EXPOSURE_TIMING),
    MetadataRequirement(SPA.META_REFERENCE_TIMESTAMP; required=true),
    CHOPPING_METADATA...,
)

mutable struct CameraStream{T}
    camera::String
    region::Tuple{UnitRange{Int},UnitRange{Int}}
    frame::Matrix{T}
    domain::AcquisitionDomain
    duration::UInt64
    sequence::UInt64
end

mutable struct CommandStream
    mirror::String
    scale::Float32
    staged::Vector{Float32}
    active::Vector{Float32}
    receipt::Union{Nothing,PipeWireArrayReceipt}
end

# Keep the mirror vectors for one originating acquisition per slot. A newer
# sequence can reuse its slot after eight acquisitions if a member was lost.
mutable struct CommandSet
    sequence::UInt64
    values::Vector{Vector{Float32}}
    present::BitVector
end

mutable struct CommandGroup
    pending::Vector{CommandSet}
    sequence::UInt64
    applied::UInt64
end

CommandGroup(inputs) = CommandGroup(
    [CommandSet(0, [zero(input.staged) for input in inputs], falses(length(inputs))) for _ in 1:8], 0, 0)

"""Independently clocked cameras and commands opened by `with_cameras`."""
mutable struct CameraSession{O,P,C,G}
    transport::O
    plant::Py
    pupil_opd_m::P
    cameras::Vector{C}
    inputs::Vector{CommandStream}
    commands::Dict{String,Vector{Float32}}
    timeout::Float64
    generation::UInt64
    group::G
end

function stream_options(spec, shape; metadata=ACQUISITION_METADATA, element=NdArray.F32_LE, properties=())
    rate = SPA.Fraction(spec["rate"]...)
    return PipeWireArrayStreamOptions(; node_name=spec["node_name"], media_name=spec["node_name"],
        schema=spec["schema"], format=NdArrayFormat(element, shape; layout=NdArray.ROW_MAJOR, rate),
        metadata, buffers=3, properties)
end

function camera_stream(plant, spec; single=false)
    name = spec["camera"]
    detector = plant.cameras[name]
    full_shape, T = camera_format(detector)
    encoding = get(spec, "encoding", "float32")
    encoding in ("float32", "uint16") || throw(ArgumentError("Invalid SPECULA camera encoding"))
    frame_type = encoding == "uint16" ? UInt16 : Float32
    element = encoding == "uint16" ? NdArray.U16_LE : NdArray.F32_LE
    (T === Float32 || frame_type === UInt16) ||
        throw(ArgumentError("$name produces UInt16 camera data"))
    shape = Tuple(Int.(spec["shape"]))
    origin = get(spec, "roi_origin", [0, 0])
    region = ntuple(axis -> (origin[axis] + 1):(origin[axis] + shape[axis]), 2)
    all(axis -> !isempty(region[axis]) && first(region[axis]) >= 1 && last(region[axis]) <= full_shape[axis], 1:2) ||
        throw(ArgumentError("$name ROI exceeds the SPECULA camera"))
    duration = pyconvert(UInt64, detector.exposure_ns)
    duration == spec["exposure_duration_ns"] || throw(ArgumentError("$name exposure duration differs from SPECULA"))
    num, denom = spec["rate"]
    cadence = single ? pyconvert(Int64, plant.period_ns) : duration
    single && duration > cadence && throw(ArgumentError("$name must expose every SPECULA step"))
    Int128(cadence) * num == 1_000_000_000 * Int128(denom) ||
        throw(ArgumentError("$name rate differs from its exposure cadence"))
    id = uuid4().value
    domain = AcquisitionDomain(ntuple(i -> UInt8((id >> (8(i - 1))) & 0xff), 16))
    camera = CameraStream(name, region, zeros(frame_type, shape), domain, duration, UInt64(0))
    metadata = get(spec, "chopped", false) ? (ACQUISITION_METADATA..., CHOPPING_METADATA...) : ACQUISITION_METADATA
    properties = single ? ("pipewireao.header.pts.clock" => "monotonic",) : ()
    return camera, stream_options(spec, shape; metadata, element, properties)
end

function command_stream(plant, spec)
    name = spec["mirror"]
    stage = command_buffer(plant, name, spec["count"])
    scale = Float32(spec["scale"])
    isfinite(scale) && !iszero(scale) || throw(ArgumentError("Invalid command scale for $name"))
    return CommandStream(name, scale, stage, zero(stage), nothing),
        stream_options(spec, size(stage); metadata=COMMAND_METADATA)
end

function single_camera_ports(wire)
    rate = wire["rate"]
    camera = Dict("camera" => wire["camera"], "node_name" => wire["frame_node_name"],
        "shape" => wire["frame_shape"], "schema" => wire["frame_schema"],
        "encoding" => wire["frame_encoding"], "exposure_duration_ns" => wire["exposure_duration_ns"], "rate" => rate)
    inputs = if haskey(wire, "commands")
        [merge(input, Dict("rate" => rate)) for input in wire["commands"]]
    else
        [Dict("mirror" => wire["mirror"], "node_name" => wire["command_node_name"],
            "count" => wire["command_count"], "schema" => wire["command_schema"],
            "scale" => wire["command_scale"], "rate" => rate)]
    end
    return [camera], inputs
end

"Open the independently clocked camera and command ports on an explicit private socket."
function with_cameras(f, plant::Py, wire::AbstractDict, remote; pupil_opd_m=nothing, active=true)
    isabspath(remote) || throw(ArgumentError("Use the absolute path of the private PipeWire socket"))
    single = !haskey(wire, "cameras")
    camera_specs, command_specs = single ? single_camera_ports(wire) : (wire["cameras"], wire["commands"])
    cameras = map(spec -> camera_stream(plant, spec; single), camera_specs)
    inputs = map(spec -> command_stream(plant, spec), command_specs)
    names = [camera.camera for (camera, _) in cameras]
    !isempty(names) && allunique(names) || throw(ArgumentError("Select distinct SPECULA cameras"))
    isempty(inputs) && throw(ArgumentError("Select at least one SPECULA command input"))
    single && !allunique(input.mirror for (input, _) in inputs) &&
        throw(ArgumentError("A single-camera command group requires distinct mirrors"))
    options = (map(last, cameras)..., map(last, inputs)...)
    directions = (map(_ -> false, cameras)..., map(_ -> true, inputs)...)
    receivers = map(first, inputs)
    commands = Dict(input.mirror => zero(input.active) for input in receivers)
    group = single ? CommandGroup(receivers) : nothing
    timeout = Float64(wire["timeout_ns"]) / 1e9
    isfinite(timeout) && timeout > 0 || throw(ArgumentError("A positive PipeWire timeout is required"))
    return with_array_streams(options; remote, input=directions, active) do transport
        session = CameraSession(transport, plant, pupil_opd_m, map(first, cameras), receivers,
            commands, timeout, UInt64(1), group)
        single && reset_pipewire_arrays!(transport; identity=AcquisitionIdentity(only(session.cameras).domain, 1, 0))
        f(session)
    end
end

"""Wait at most the declared timeout for the camera links to stream."""
function wait_for_streams!(session::CameraSession)
    transport = session.transport
    ports = session.group === nothing ? eachindex(transport.streams) : eachindex(session.cameras)
    is_ready(index) = with_array_stream(transport, index) do stream
        stream_state(stream) == LibPipeWire.PW_STREAM_STATE_STREAMING
    end
    timedwait(() -> all(is_ready, ports), session.timeout; pollint=0.002) == :ok ||
        error("SPECULA links did not reach streaming state")
    return nothing
end

function receive_command!(session, input, index)
    port = length(session.cameras) + index
    array_stream_status(session.transport, port).failed && error("SPECULA DM input failed")
    receipt = drain_pipewire_array!(input.staged, session.transport, port)
    if receipt !== nothing
        all(value -> isfinite(value * input.scale), input.staged) || error("SPECULA DM command is not finite")
    end
    return receipt
end

# Independent absolute offsets from distinct ports add on the same mirror.
function receive_commands!(session::CameraSession, ::Nothing)
    for (index, input) in enumerate(session.inputs)
        receipt = receive_command!(session, input, index)
        isnothing(receipt) && continue
        input.active .= input.scale .* input.staged
        input.receipt = receipt
    end
    foreach(values -> fill!(values, 0), values(session.commands))
    for input in session.inputs
        session.commands[input.mirror] .+= input.active
    end
    all(command -> all(isfinite, command), values(session.commands)) || error("Combined DM command is not finite")
    return nothing
end

function receive_commands!(session::CameraSession, group::CommandGroup)
    camera = only(session.cameras)
    for (index, input) in enumerate(session.inputs)
        receipt = receive_command!(session, input, index)
        receipt === nothing && continue
        identity = receipt.identity
        identity === nothing && continue
        identity.domain == camera.domain && identity.generation == session.generation &&
            group.sequence < identity.sequence <= camera.sequence || continue
        slot = group.pending[mod1(identity.sequence, length(group.pending))]
        slot.sequence <= identity.sequence || continue
        if slot.sequence < identity.sequence
            slot.sequence = identity.sequence
            fill!(slot.present, false)
        end
        slot.values[index] .= input.scale .* input.staged
        slot.present[index] = true
    end
    latest = nothing
    for slot in group.pending
        if slot.sequence > group.sequence && all(slot.present) &&
                (latest === nothing || slot.sequence > latest.sequence)
            latest = slot
        end
    end
    latest === nothing && return nothing
    for (input, command) in zip(session.inputs, latest.values)
        copyto!(session.commands[input.mirror], command)
    end
    group.sequence = latest.sequence
    group.applied += 1
    return nothing
end

"""Drain eligible RTC commands and adopt completed command groups."""
receive_commands!(session::CameraSession) = receive_commands!(session, session.group)

copy_pixels!(frame::Matrix{T}, pixels::AbstractMatrix{T}) where {T} = copyto!(frame, pixels)
function copy_pixels!(frame::Matrix{UInt16}, pixels::AbstractMatrix{Float32})
    return map!(frame, pixels) do pixel
        isfinite(pixel) && 0 <= pixel <= typemax(UInt16) ||
            throw(ArgumentError("UInt16 transport requires finite raw ADC codes in 0:65535"))
        round(UInt16, pixel)
    end
end

function await_camera_publication!(flushed, timeout)
    strategy = SimRunner.current_strategy()
    if SimRunner.name(strategy) == "yielding"
        timedwait(flushed, timeout; pollint=0.002) == :ok || error("SPECULA camera publication timed out")
        return nothing
    end
    # Let the native data loop progress between trigger and status checks.
    start = time_ns()
    retry_ns = UInt64(0)
    while true
        elapsed_ns = time_ns() - start
        if elapsed_ns >= retry_ns
            flushed() && return nothing
            retry_ns = elapsed_ns + UInt64(2_000_000)
        end
        elapsed_ns / 1e9 < timeout || error("SPECULA camera publication timed out")
        SimRunner.idle!(strategy)
    end
end

"Advance one model step and publish each camera that completed an exposure."
function step!(session::CameraSession)
    receive_commands!(session)
    exposures = session.plant.step(; pupil_opd_m=session.pupil_opd_m, commands=pydict(session.commands))
    published = Int[]
    for (index, camera) in enumerate(session.cameras)
        pyconvert(Bool, exposures.__contains__(camera.camera)) || continue
        exposure = exposures[camera.camera]
        copy_pixels!(camera.frame, view(PyArray(exposure.pixels), camera.region...))
        chopped = !pyis(exposure.reference_open, pybuiltins.None)
        fringed = chopped && pyconvert(Bool, exposure.reference_open)
        optical = chopped ? (fringed ? OPTICAL_FRINGED : OPTICAL_UNFRINGED) : nothing
        trigger = chopped ? (fringed ? ACQUISITION_TRIGGER_HIGH : ACQUISITION_TRIGGER_LOW) : ACQUISITION_TRIGGER_ABSENT
        camera.sequence += 1
        identity = AcquisitionIdentity(camera.domain, session.generation, camera.sequence)
        pts = session.group === nothing ? pyconvert(Int64, exposure.end_ns) : Int64(time_ns())
        staged = stage_pipewire_array!(session.transport, index, camera.frame;
            identity, pts, duration=camera.duration, trigger_state=trigger,
            has_trigger_classification=chopped, optical_classification=optical)
        staged && push!(published, index)
        if session.group !== nothing
            status = array_stream_status(session.transport, index)
            (status.failed || !iszero(status.rejected)) && error("Could not publish $(camera.camera)")
        elseif !staged
            error("Could not stage $(camera.camera)")
        end
    end
    if session.group !== nothing
        trigger_process!(session.transport)
        return published
    end
    isempty(published) && return published
    function flushed()
        trigger_process!(session.transport)
        statuses = map(index -> array_stream_status(session.transport, index), published)
        any(status -> status.failed, statuses) && error("SPECULA camera stream failed")
        return all(status -> !status.pending, statuses)
    end
    await_camera_publication!(flushed, session.timeout)
    return published
end

"""
    reset_pipewire_hil!(session::CameraSession, fresh_plant)

Install a fresh instance of the same plant while stopped, clear the command
history and return the new acquisition generation.
"""
function reset_pipewire_hil!(session::CameraSession, fresh_plant::Py)
    generation = Base.checked_add(session.generation, UInt64(1))
    if session.group === nothing
        for (index, camera) in enumerate(session.cameras)
            identity = AcquisitionIdentity(camera.domain, generation, UInt64(0))
            reset_pipewire_array!(session.transport, index; identity)
        end
        for (index, command) in enumerate(session.inputs)
            reset_pipewire_array!(session.transport, length(session.cameras) + index)
            fill!(command.active, 0)
            command.receipt = nothing
        end
    else
        identity = AcquisitionIdentity(only(session.cameras).domain, generation, UInt64(0))
        reset_pipewire_arrays!(session.transport; identity)
        for slot in session.group.pending
            slot.sequence = 0
            fill!(slot.present, false)
        end
        session.group.sequence = 0
        session.group.applied = 0
    end
    foreach(camera -> camera.sequence = 0, session.cameras)
    foreach(command -> fill!(command, 0), values(session.commands))
    session.plant = fresh_plant
    session.generation = generation
    return generation
end
