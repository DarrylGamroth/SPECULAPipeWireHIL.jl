using SPECULAPipeWireHIL
using AdaptiveOpticsSim.AlgorithmGraphs:
    adopt_hil_command!, hil_boundary_status, hil_command_buffer, hil_frame_buffer,
    reset_hil_boundary!, step_hil_frame!
using AdaptiveOpticsSimPipeWireHIL: frame_acquisition_generation
import AdaptiveOpticsSimPipeWireHIL
using Aqua
using PipeWireAO
using PipeWireAO_jll
using PythonCall
using Test

const Adapter = SPECULAPipeWireHIL
const FIXTURE = let sys = pyimport("sys")
    sys.path.insert(0, @__DIR__)
    try
        pyimport("plant").Plant
    finally
        sys.path.pop(0)
    end
end

const FRAME_SCHEMA = "test/specula-camera/1"
const COMMAND_SCHEMA = "pipewireao/dm-command-opd-um-f32/1"

function wire(; adc=true)
    return Dict{String,Any}(
        "camera" => "wfs", "mirror" => "primary",
        "frame_node_name" => "specula-wfs", "command_node_name" => "specula-primary",
        "frame_shape" => (adc ? [3, 3] : [2, 3]), "frame_encoding" => (adc ? "uint16" : "float32"),
        "frame_schema" => FRAME_SCHEMA, "command_schema" => COMMAND_SCHEMA,
        "command_count" => 2, "command_scale" => 1.0f0, "rate" => [500, 1],
        "exposure_duration_ns" => 2_000_000, "timeout_ns" => 5_000_000_000,
    )
end

function named_wire()
    document = wire()
    document["commands"] = [
        Dict("mirror" => "primary", "node_name" => "specula-primary",
            "schema" => COMMAND_SCHEMA, "count" => 2, "scale" => 2.0f0),
        Dict("mirror" => "secondary", "node_name" => "specula-secondary",
            "schema" => COMMAND_SCHEMA, "count" => 1, "scale" => 0.5f0),
    ]
    return document
end

@testset "Package quality" begin
    Aqua.test_all(Adapter)
end

@testset "Common lockstep plant interface" begin
    @test Adapter.prepare_pipewire_hil === AdaptiveOpticsSimPipeWireHIL.prepare_pipewire_hil
    plant = FIXTURE()
    boundary, cfg = Adapter.prepare_boundary(plant, wire(), "/test-core")
    @test cfg isa PipeWireHILConfiguration
    @test size(hil_frame_buffer(boundary)) == (3, 3)
    @test eltype(hil_frame_buffer(boundary)) == UInt16
    @test hil_boundary_status(boundary).frame_sequence == 0
    @test_throws ErrorException adopt_hil_command!(boundary, UInt64(1))
    @test step_hil_frame!(boundary) == 1
    @test all(==(20), hil_frame_buffer(boundary))
    @test_throws ErrorException step_hil_frame!(boundary)
    @test_throws ErrorException adopt_hil_command!(boundary, UInt64(2))
    hil_command_buffer(boundary) .= [3, NaN]
    @test_throws ErrorException adopt_hil_command!(boundary, UInt64(1))
    @test hil_boundary_status(boundary).command_response_required
    hil_command_buffer(boundary) .= [3, 4]
    @test adopt_hil_command!(boundary, UInt64(1)) == 1
    hil_command_buffer(boundary) .= 17
    @test step_hil_frame!(boundary) == 2
    @test all(==(27), hil_frame_buffer(boundary))
    reset_hil_boundary!(boundary)
    @test all(iszero, hil_frame_buffer(boundary)) && all(iszero, hil_command_buffer(boundary))
    @test hil_boundary_status(boundary) ==
        (; frame_sequence=0, active_command_sequence=0, command_response_required=false, failed=false)

    for fault in ("shape", "dtype", "raise")
        plant.fault = fault
        @test_throws Exception step_hil_frame!(boundary)
        @test hil_boundary_status(boundary).failed
        @test_throws ErrorException step_hil_frame!(boundary)
        @test_throws ErrorException adopt_hil_command!(boundary, UInt64(1))
        reset_hil_boundary!(boundary)
    end

    plant = FIXTURE()
    named, cfg = Adapter.prepare_boundary(plant, named_wire(), "/test-core")
    @test cfg.command_scale == (; primary=2f0, secondary=0.5f0)
    @test step_hil_frame!(named) == 1
    named.command.primary .= 2
    named.command.secondary .= Inf
    @test_throws ErrorException adopt_hil_command!(named, UInt64(1))
    @test all(iszero, named.active_command.primary)
    named.command.secondary .= 3
    @test adopt_hil_command!(named, UInt64(1)) == 1
    @test step_hil_frame!(named) == 2
    @test all(==(27), named.frame)
    reset_hil_boundary!(named)
    @test all(iszero, named.active_command.secondary)

    floating, _ = Adapter.prepare_boundary(FIXTURE(false), wire(; adc=false), "/test-core")
    @test hil_frame_buffer(floating) == zeros(Float32, 2, 3)
end

@testset "Plant and wire admission" begin
    declaration = wire()
    plant = FIXTURE()
    @test Adapter.acquisition_mode(declaration) == "lockstep"
    @test Adapter.acquisition_mode(declaration, "free-running") == "free-running"
    @test Adapter.acquisition_mode(Dict("cameras" => [])) == "free-running"
    @test_throws ArgumentError Adapter.acquisition_mode(declaration, "unknown")
    @test_throws ArgumentError Adapter.acquisition_mode(Dict("cameras" => []), "lockstep")
    @test_throws ArgumentError Adapter.prepare_boundary(plant, declaration, "default")
    for (field, value) in ("command_count"=>3, "rate"=>[400, 1],
            "exposure_duration_ns"=>1_000_000, "frame_shape"=>[2, 3], "frame_encoding"=>"float32")
        @test_throws ArgumentError Adapter.prepare_boundary(
            plant, merge(declaration, Dict(field=>value)), "/test-core",
        )
    end
    plant.cameras["wfs"].exposure_ns = 4_000_000
    @test_throws ArgumentError Adapter.prepare_boundary(
        plant, merge(declaration, Dict("exposure_duration_ns"=>4_000_000)), "/test-core",
    )
    plant = FIXTURE()
    cameras, commands = Adapter.single_camera_ports(declaration)
    camera = only(cameras)
    for (field, value) in ("encoding"=>"bad", "encoding"=>"float32", "shape"=>[4, 3],
            "rate"=>[400, 1], "exposure_duration_ns"=>1_000_000, "roi_origin"=>[-1, 0])
        @test_throws ArgumentError Adapter.camera_stream(plant, merge(camera, Dict(field=>value)); single=true)
    end
    @test_throws ArgumentError Adapter.command_stream(plant, merge(only(commands), Dict("scale"=>0)))
    @test_throws ArgumentError Adapter.with_cameras(identity, plant, declaration, "default")
    for candidate in (
            merge(declaration, Dict("timeout_ns"=>0)),
            Dict("cameras"=>[camera, camera], "commands"=>commands),
            Dict("cameras"=>[camera], "commands"=>[]))
        @test_throws ArgumentError Adapter.with_cameras(identity, plant, candidate, "/test-core")
    end
    dst = zeros(UInt16, 2, 2)
    Adapter.copy_pixels!(dst, fill(5.4f0, 2, 2))
    @test all(==(5), dst)
    @test_throws ArgumentError Adapter.copy_pixels!(dst, fill(-1f0, 2, 2))
    @test_throws ArgumentError Adapter.copy_pixels!(dst, fill(NaN32, 2, 2))
end

include("private_core.jl")
