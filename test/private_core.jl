using AdaptiveOpticsSimPipeWireHIL: array_stream_status, drain_pipewire_array!,
    stage_pipewire_array!, with_array_streams

function await_hil(ready)
    timedwait(ready, 12; pollint=0.005) == :ok || error("Timed out on the disposable PipeWire core")
end

function with_private_core(f)
    mktempdir() do temporary
        runtime = mkdir(joinpath(temporary, "runtime"); mode=0o700)
        config = mkdir(joinpath(temporary, "config"))
        name = "specula-fixture"
        remote = joinpath(runtime, name)
        common = """
        context.spa-libs = { support.* = support/libspa-support }
        context.modules = [
            { name = libpipewire-module-protocol-native }
            { name = libpipewire-module-client-node }
        """
        write(joinpath(config, "server.conf"),
            "context.properties = { core.daemon = true core.name = $name support.dbus = false }\n" *
            common * """
                { name = libpipewire-module-scheduler-v1 }
                { name = libpipewire-module-link-factory }
                { name = libpipewire-module-access }
            ]
            """)
        write(joinpath(config, "client.conf"),
            "context.properties = { support.dbus = false }\n" * common * "]\n")
        sdk = String(PipeWireAO_jll.artifact_dir)
        executable = joinpath(sdk, "bin", "pipewire-ao")
        libdir = dirname(PipeWireAO_jll.libpipewire_ao_path)
        environment = (
            "XDG_RUNTIME_DIR" => runtime, "PIPEWIREAO_RUNTIME_DIR" => runtime,
            "PIPEWIREAO_CONFIG_DIR" => config,
            "PIPEWIREAO_MODULE_DIR" => joinpath(libdir, "pipewire-ao-0.3"),
            "PIPEWIREAO_SPA_PLUGIN_DIR" => joinpath(libdir, "spa-ao-0.2"),
        )
        withenv(environment...) do
            daemon = run(`$executable -c server.conf`; wait=false)
            try
                await_hil(() -> issocket(remote))
                f(remote)
            finally
                process_exited(daemon) || kill(daemon)
                wait(daemon)
            end
        end
    end
end

function endpoint(registry, name, direction)
    nodes = find_globals(registry; interface="PipeWire:Interface:Node",
        properties=("node.name" => name,))
    isempty(nodes) && return nothing
    node = only(nodes).id
    ports = find_globals(registry; interface="PipeWire:Interface:Port",
        properties=("node.id" => string(node), "port.direction" => direction))
    return isempty(ports) ? nothing : (node, only(ports).id)
end

function with_links(f, owner, routes)
    registry = with_thread_loop_lock(_ -> Registry(owner.core), owner.loop)
    links = Link[]
    try
        for (producer, consumer) in routes
            source = target = nothing
            await_hil() do
                roundtrip(registry)
                source = endpoint(registry, producer, "out")
                target = endpoint(registry, consumer, "in")
                return source !== nothing && target !== nothing
            end
            link = with_thread_loop_lock(owner.loop) do _
                create_object(owner.core, "link-factory", Link; properties=Dict(
                    "link.output.node" => string(source[1]), "link.output.port" => string(source[2]),
                    "link.input.node" => string(target[1]), "link.input.port" => string(target[2]),
                    "object.linger" => "false"))
            end
            push!(links, link)
        end
        roundtrip(registry)
        return f()
    finally
        with_thread_loop_lock(owner.loop) do _
            foreach(close, reverse(links))
            close(registry)
        end
    end
end

function observe_camera(owner, peer, index, pixels)
    sample = Ref{Any}(nothing)
    await_hil() do
        trigger_process!(owner.transport)
        sample[] = drain_pipewire_array!(pixels, peer, index)
        return sample[] !== nothing
    end
    return sample[]
end

function send_command(peer, port, owner, input, identity, values)
    @test stage_pipewire_array!(peer, port, values; identity, pts=Int64(time_ns()),
        duration=UInt64(2_000_000))
    await_hil() do
        trigger_process!(peer)
        return array_stream_status(owner.transport, length(owner.cameras) + input).pending
    end
end

function grouped_cameras(remote)
    declaration = named_wire()
    cameras, commands = Adapter.single_camera_ports(declaration)
    camera = only(cameras)
    peer_camera = merge(camera, Dict("node_name" => "peer-camera"))
    peers = [merge(command, Dict("node_name"=>"peer-" * command["mirror"])) for command in commands]
    options = (Adapter.stream_options(peer_camera, (3, 3); element=NdArray.U16_LE),
        (Adapter.stream_options(command, (command["count"],); metadata=Adapter.COMMAND_METADATA)
            for command in peers)...)
    routes = ((camera["node_name"], peer_camera["node_name"]),
        ((output["node_name"], input["node_name"]) for (output, input) in zip(peers, commands))...)
    pixels = zeros(UInt16, 3, 3)
    Adapter.with_cameras(FIXTURE(), declaration, remote) do owner
        with_array_streams(options; remote, input=(true, false, false)) do peer
            with_links(peer, routes) do
                Adapter.wait_for_streams!(owner)
                function expose()
                    @test Adapter.step!(owner) == [1]
                    return observe_camera(owner, peer, 1, pixels)
                end
                first_frame = expose()
                @test all(==(20), pixels)
                @test first_frame.duration == 2_000_000
                first_id = first_frame.identity
                @test first_id.sequence == 1 && first_id.generation == 1
                send_command(peer, 2, owner, 1, first_id, Float32[2, 3])
                second_frame = expose()
                @test all(==(20), pixels)
                @test owner.group.applied == 0
                send_command(peer, 3, owner, 2, first_id, Float32[8])
                @test expose().identity.sequence == second_frame.identity.sequence + 1
                @test owner.group.applied == 1
                @test all(==(34), pixels)
                @test !stage_pipewire_array!(peer, 2, Float32[9, 9]; identity=first_id,
                    pts=Int64(time_ns()), duration=UInt64(2_000_000))
                expose()
                @test all(==(34), pixels)
                @test reset_pipewire_hil!(owner, FIXTURE()) == 2
                reset = expose()
                @test reset.identity == AcquisitionIdentity(first_id.domain, 2, 1)
                @test all(==(20), pixels)
            end
        end
    end
end

function independent_cameras(remote)
    cameras, commands = Adapter.single_camera_ports(wire(; adc=false))
    camera = merge(only(cameras), Dict("chopped"=>true, "shape"=>[2, 2], "roi_origin"=>[0, 1]))
    declaration = Dict("cameras"=>[camera], "commands"=>commands, "timeout_ns"=>10_000_000_000)
    peer_camera = merge(camera, Dict("node_name"=>"peer-wfs"))
    peer_command = merge(only(commands), Dict("node_name"=>"peer-command"))
    options = (Adapter.stream_options(peer_camera, (2, 2);
            metadata=(Adapter.ACQUISITION_METADATA..., Adapter.CHOPPING_METADATA...)),
        Adapter.stream_options(peer_command, (2,); metadata=Adapter.COMMAND_METADATA))
    routes = ((camera["node_name"], peer_camera["node_name"]),
        (peer_command["node_name"], only(commands)["node_name"]))
    plant = FIXTURE(false)
    pixels = zeros(Float32, 2, 2)
    Adapter.with_cameras(plant, declaration, remote) do owner
        with_array_streams(options; remote, input=(true, false)) do peer
            with_links(peer, routes) do
                Adapter.wait_for_streams!(owner)
                plant.skip = true
                @test isempty(Adapter.step!(owner))
                plant.skip = false
                plant.reference_open = true
                @test Adapter.step!(owner) == [1]
                result = observe_camera(owner, peer, 1, pixels)
                @test result.optical_classification == OPTICAL_FRINGED
                @test result.pts == 4_000_000
                send_command(peer, 2, owner, 1, result.identity, Float32[2, 3])
                plant.reference_open = false
                @test Adapter.step!(owner) == [1]
                @test observe_camera(owner, peer, 1, pixels).optical_classification == OPTICAL_UNFRINGED
                @test all(==(25), pixels)
                @test reset_pipewire_hil!(owner, FIXTURE(false)) == 2
                @test owner.inputs[1].receipt === nothing
                @test all(iszero, owner.commands["primary"])
            end
        end
    end
end

@testset "Disposable PipeWire core" begin
    with_private_core() do remote
        for declaration in (wire(), named_wire(), merge(wire(), Dict("command_schema"=>"test/command/1")))
            prepared = Adapter.prepare(FIXTURE(), declaration, remote)
            try
                @test prepared isa PreparedPipeWireHIL
                @test frame_acquisition_generation(prepared) == 1
                fresh = FIXTURE()
                @test reset_pipewire_hil!(prepared, fresh) === prepared
                @test frame_acquisition_generation(prepared) == 2
                @test pyis(prepared.boundary.plant, fresh)
            finally
                close(prepared)
            end
        end
        grouped_cameras(remote)
        independent_cameras(remote)
    end
end
