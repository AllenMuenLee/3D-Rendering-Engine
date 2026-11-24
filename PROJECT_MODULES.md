# 3D Rendering Engine — Simplified Architecture

## High-Level Flow

```
top.v
 ├── object_lib/keyboard_v2.v  (PS/2 multi-key WASD receiver from starter library)
 ├── 0.05s key counter         (Updates angle_x / angle_z every 0.05s while pressed)
 ├── renderer.v                (Simple 2-stage solid-face 3D triangle renderer)
 │    ├── shape_lut.v          (AI-generated triangle vertex & normal ROM from shape_lut.hex)
 │    └── sin_lut.v            (AI-generated sine/cosine ROM)
 └── object_lib/vga_adapter.v  (DESim VGA module when `define DESIM is set)
      [or vga_board_adapter.v  (Physical FPGA VGA timing + framebuffer)]
```

---

## Custom Files (Root Directory)

### `top.v`
- Connects board/DESim I/O (`CLOCK_50`, `SW[9:0]`, `KEY[0]`, `LEDR`, `PS2_CLK/DAT`, VGA).
- Instantiates `keyboard` from `object_lib/keyboard_v2.v` to get `wasd_held`.
- Uses a `0.05 s` counter (`KEY_DELAY_CYCLES = 2,500,000` @ 50 MHz) to update `angle_x` (W/S) and `angle_z` (A/D) while keys are held.
- Decodes `SW[9:0]` one-hot into 6 selectable shapes (`SW[9:5]` Platonic solids, `SW[3]` NVIDIA logo; all off = blank screen).
- Instantiates `renderer` and connects its pixel output `(vga_x, vga_y, vga_color, vga_write)` directly to `object_lib/vga_adapter.v` (DESim) or `vga_board_adapter.v` (DE1-SoC hardware).

### `renderer.v`
Simple 4-state FSM (`S_IDLE` → `S_MATRIX_MUL` → `S_CLEAR` → `S_RENDER`):
1. **Trigger:** Whenever `angle_x != prev_angle_x`, `angle_z != prev_angle_z`, or the selected shape changes, computes the 3×3 rotation matrix `R = Rz(angle_z) * Rx(angle_x)` from `sin_lut`.
2. **Stage 1 (`S_MATRIX_MUL`):** Loops through all `num_tris` entries in `shape_lut`, multiplies every triangle's 3 vertices `(v0, v1, v2)` and surface normal `n` by `R`, and stores the results in `updated_lut`.
3. **Screen Clear (`S_CLEAR`):** Clears the previous frame to black before drawing the new one.
4. **Stage 2 (`S_RENDER`):** Loops through all entries in `updated_lut`. For each triangle:
   - Uses `if (cur_ny > 0)` to skip back-facing triangles.
   - Computes `D = LIGHT_POS - centroid` and takes the direct dot product `dot(N, D)`, scaling it linearly into 10%~90% brightness (`26..230`).
   - Scans the triangle's 2D bounding box and fills every pixel inside the triangle solidly.

### AI-Generated LUTs & Generator
- `generate_shape_lut.py` — Packs all triangles `(v0, v1, v2, n)` for the 5 Platonic solids + NVIDIA logo from `platonic_solids.json` into `shape_lut.hex` (108-bit / 27-hex-digit words, signed 9-bit Q2.7 where `1.0 = 128`).
- `shape_lut.v` + `shape_lut.hex` — ROM holding all 6 shapes (`1,536` entries = 6 × 256).
- `sin_lut.v` — Sine/cosine ROM (`0°..359°`, Q2.14).
- `vga_board_adapter.v` — Physical FPGA VGA controller (used when `DESIM` is not defined).

### Verification
- `ModelSim/testbench_simple.v` — Verifies shape switching, 0.05s keyboard rotation + matrix update, shaded pixel output, and blanking.
