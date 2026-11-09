# Raw physics replay fixtures

`physics-fixed32.bin` and `physics-fixed64.bin` contain `M3RPLY02` raw constructor
inputs and tick-indexed commands. Each paired `.hashes` file holds tick zero
through tick 1,440, generated with the versioned solver and independently replayed
on x86-64, ARMv5TE and RISC-V64. See the [format, regeneration and verification
contract](../../docs/physics-determinism.md).

Version 02 replaces dynamic-support sleeping with static-support-only sleep,
removes the wake-speed configuration word and reverses gravity at tick 900.
Earlier version-01 packets are intentionally rejected by the current reader.

The static triangles derive from the existing teapot and Stanford Bunny soup
fixtures, scaled/translated as in `examples/physics/scene.rs`. Credits: Martin
Newell / University of Utah (Utah Teapot), Stanford University Computer Graphics
Laboratory / Greg Turk and Marc Levoy (Stanford Bunny). Their source-model terms
and attribution are retained in the [mesh notice](../../examples/collision/assets/NOTICE.md).
The tiled floor and dynamic reference-body/input setup are repository examples.
