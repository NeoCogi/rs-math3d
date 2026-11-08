# Mesh sources and attribution

The `.soup` files are benchmark data derived from the following models. Their
source terms are separate from the repository's BSD source-code license.

`bunny.soup` derives from `bunny/reconstruction/bun_zipper_res3.ply` in the
[Stanford 3D Scanning Repository](https://graphics.stanford.edu/data/3Dscanrep/).
Credit: Stanford University Computer Graphics Laboratory; Greg Turk and Marc
Levoy's Stanford Bunny reconstruction. The repository permits research use and
free redistribution with attribution, and specifically states: “You can do
anything you want to the Stanford bunny or the armadillo.” Consult that source
page for its full model-use terms. The selected reconstruction has 1,889
vertices and 3,851 triangles; it is not the full-resolution reconstruction.

`teapot.soup` derives from the Utah Teapot OBJ distributed with
[Stanford CS148's example code](https://graphics.stanford.edu/courses/cs148-10-summer/as3/code/as3/teapot.obj).
Credit: the historic Utah Teapot developed by Martin Newell at the University
of Utah. The [official University of Utah teapot page](https://graphics.cs.utah.edu/teapot/)
provides model-use terms allowing reuse, including commercial use, and requests
identification and acknowledgment of the model's historical origin. This OBJ
contains 3,644 vertices and 6,320 triangles.

Both fixtures are modified versions: centered, uniformly rescaled to a longest
extent of 32, and quantized to a 1/256 grid. Winding and triangle order are
preserved. Source files and resulting binary fixtures are pinned by SHA256 in
`manifest.json`; the preparation tool records any collapsed faces removed.
Neither fixture lost a triangle under the current preparation settings.

Regenerate from the pinned online sources:

```sh
python3 tools/prepare_collision_meshes.py --download
```

Or provide previously downloaded source files with `--bunny-archive PATH` and
`--teapot-obj PATH`. The tool validates their hashes before processing them.
Ordinary Rust tests and benchmarks use the checked-in fixtures without network
access, external assets or Python.
