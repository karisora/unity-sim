# ERC Landmark Textures

These assets map the landmark number to the ArUco `5x5_1000` ID as follows:

| Number | ArUco ID | Number | ArUco ID |
|---:|---:|---:|---:|
| 1 | 51 | 8 | 58 |
| 2 | 52 | 9 | 59 |
| 3 | 53 | 10 | 60 |
| 4 | 54 | 11 | 61 |
| 5 | 55 | 12 | 62 |
| 6 | 56 | 13 | 63 |
| 7 | 57 | 14 | 64 |
| 15 | 65 |  |  |

## Dimensions represented by each texture

- Full face: **250 x 320 mm** (`700 x 896 px`)
- Number box: **150 x 70 mm**, positioned 50 mm from the left and 20 mm from the top
- ArUco marker: **150 x 150 mm**, positioned 50 mm from the left and 120 mm from the top
- Everything outside the black number, number-box outline, and marker is pure white (`RGB 255,255,255`)

The marker is rendered directly from the matching SVG in `Assets/ArUco`; it is not regenerated from a separate dictionary.

## Unity use

The `Landmark` root in `Assets/Simulation/Scenes/ERC2025.unity` has a
`LandmarkSetUprightFaces` component. When the scene opens (and at runtime), it
automatically assigns numbers 1-15 to children `L1`-`L15` and replaces their
visual Cube meshes with upright UVs. Existing transforms and colliders are kept.

For a landmark created outside that hierarchy, use the included custom mesh:

Unity's built-in Cube has one side with vertically reversed UVs. Use the included
`LandmarkBox_Upright.obj` mesh to keep every vertical face upright:

1. Select the existing `250 x 320 x 250 mm` Cube.
2. In its `Mesh Filter`, replace the built-in `Cube` mesh with the mesh contained in `LandmarkBox_Upright.obj`.
3. Keep the Transform scale at `X=0.25`, `Y=0.32`, `Z=0.25` metres.
4. Drag the required `Landmark_XX_IDYY.mat` onto its `Mesh Renderer`.

The four vertical faces now have matching upright, non-mirrored UVs. The top and bottom sample a
white area of the same texture and therefore remain blank. The custom mesh is a
unit cube, so the existing Box Collider and physical dimensions do not need to change.

Textures use point filtering, clamp wrapping, no mipmaps, and no compression so the marker modules stay sharp.

Run `python3 Tools/generate_landmark_textures.py` to rebuild all textures and materials from the SVG source files.
