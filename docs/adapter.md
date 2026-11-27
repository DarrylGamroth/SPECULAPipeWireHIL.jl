# Adapter contract

The Julia owner task calls the Python plant. PipeWire callbacks handle prepared
host buffers and the native array metadata. A caller selects an absolute
PipeWire socket and manages graph links, startup, stop, and closure.

## Plant

The plant exposes a positive `period_ns`, a mapping `command_counts`, and a
`cameras` mapping. A camera exposes `exposure_ns`,
`node.outputs["out_pixels"].pixels.shape`, and `adc`. When `adc` describes a
UInt16 detector, its `padding_rows` are included in the wire shape. Float32
camera pixels are expressed in the detector's native numerical units.

The Python method `step(*, commands, pupil_opd_m)` accepts a mapping of
mirror names to command vectors in their declared plant units.
`pupil_opd_m` is the optional entrance optical path difference in metres.
The plant returns completed exposures by camera name, each with a CPU
array `pixels`, an integer model `end_ns`, and boolean or null
`reference_open`. Lockstep requires the selected camera on every step.
Reset the Python model with a fresh instance before restarting its time base.

## Wire declaration

The wire input is a Julia dictionary. The single-camera form uses these keys.

| Keys | Meaning |
| --- | --- |
| `camera`, `frame_node_name`, `frame_schema` | Plant camera and its PipeWire endpoint |
| `frame_shape`, `frame_encoding` | Detector rows and columns, then `uint16` or `float32` |
| `rate`, `exposure_duration_ns`, `timeout_ns` | Frame rate as numerator and denominator, sensor exposure, and wait budget |
| `mirror`, `command_node_name`, `command_schema`, `command_count`, `command_scale` | DM endpoint and signed scale from wire to plant units |

A single camera can use a `commands` list in place of the single DM fields.
Each entry supplies `mirror`, `node_name`, `schema`, `count`, and `scale`.
For lockstep, the adapter stages the exposure and the shared transport accepts
one matching command set before releasing the next step.

For independent exposure cadences, supply a `cameras` list and a `commands`
list. Each camera entry supplies `camera`, `node_name`, `schema`, `shape`,
`encoding`, `exposure_duration_ns`, and its own `rate`. Optional
`roi_origin` gives zero-based row and column offsets and `chopped` enables
optical and electrical classifications. Each command adds its own `rate`.
Multiple ports for one mirror in independent-camera acquisition contribute
absolute offsets in the same plant units. A free-running single-camera
command group adopts only complete corrections for a common acquisition identity.
