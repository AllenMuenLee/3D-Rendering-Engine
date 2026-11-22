# 3D Rendering Engine — Simplified Architecture

## High-Level Flow

```
top.v
 ├── object_lib/keyboard_v2.v  (PS/2 multi-key WASD receiver from starter library)
 ├── 0.05s key counter         (Updates angle_x / angle_z every 0.05s while pressed)
 ├── renderer.v                (Simple 2-stage 3D point-cloud renderer)
 │    ├── shape_lut.v          (AI-generated surface point & normal ROM from shape_lut.hex)
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
- Decodes `SW[9:0]` one-hot into 10 selectable shapes (`SW[9:5]` Platonic solids, `SW[4:0]` custom figures; all off = blank screen).
- Instantiates `renderer` and connects its pixel output `(vga_x, vga_y, vga_color, vga_write)` directly to `object_lib/vga_adapter.v` (DESim) or `vga_board_adapter.v` (DE1-SoC hardware).

### `renderer.v`
Simple 4-state FSM (`S_IDLE` → `S_MATRIX_MUL` → `S_CLEAR` → `S_RENDER`):
1. **Trigger:** Whenever `angle_x != prev_angle_x`, `angle_z != prev_angle_z`, or the selected shape changes, computes the 3×3 rotation matrix `R = Rz(angle_z) * Rx(angle_x)` from `sin_lut`.
2. **Stage 1 (`S_MATRIX_MUL`):** Loops through all `NUM_POINTS` (2,048) entries in `shape_lut`, multiplies every point `(px, py, pz)` and surface normal `(nx, ny, nz)` by `R`, and stores the results in `updated_lut`.
3. **Screen Clear (`S_CLEAR`):** Clears the previous frame to black before drawing the new one.
4. **Stage 2 (`S_RENDER`):** Loops through all entries in `updated_lut`. For each point:
   - Computes `D = LIGHT_POS - P` and takes the direct dot product `dot(N, D)`, scaling it linearly into 10%~90% brightness (`26..230`).
   - Uses `if (cur_ny > 0)` to decide whether to display the pixel on VGA at `(320 + cur_px, 240 + cur_pz)`.

### AI-Generated LUTs & Generator
- `generate_shape_lut.py` — Samples 2,048 evenly spaced surface points & outward normals per shape from `platonic_solids.json` into `shape_lut.hex` (signed 9-bit Q2.7, `1.0 = 128`).
- `shape_lut.v` + `shape_lut.hex` — ROM holding all 10 shapes (`20,480` entries).
- `sin_lut.v` — Sine/cosine ROM (`0°..359°`, Q2.14).
- `vga_board_adapter.v` — Physical FPGA VGA controller (used when `DESIM` is not defined).

### Verification
- `ModelSim/testbench_simple.v` — Verifies shape switching, 0.05s keyboard rotation + matrix update, shaded pixel output, and blanking.
