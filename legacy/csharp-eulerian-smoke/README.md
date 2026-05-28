# Iguazu: Legacy C# Eulerian Solver

> Pronounced **eh-gwah-SOO**. This is the original C# implementation that started the Iguazu project — a 2D incompressible Eulerian smoke solver on a staggered MAC grid. It is preserved here as a complete, working artifact. Active development has moved to a C++ rewrite that targets the broader Iguazu roadmap (Eulerian, PIC, FLIP, APIC, MLS-MPM).

---

## Demos



<img width="360" height="310" alt="Screenshot 2026-05-27 193329" src="https://github.com/user-attachments/assets/342bfae3-388a-4be9-9724-ded5490dbc95" /> <img width="360" height="310" alt="Screenshot 2026-05-27 193310" src="https://github.com/user-attachments/assets/2410559a-0291-4a4e-9b12-d6ea719c4acd" />
<img width="360" height="290" alt="Screenshot 2026-05-06 214641" src="https://github.com/user-attachments/assets/de6eb5c5-62e9-4e22-a9f5-28603b030f9c" />
<img width="300" height="290" alt="Screenshot 2026-05-27 193402" src="https://github.com/user-attachments/assets/5b27e36d-a74d-4a42-8763-58822f3114af" />

---

## What this is

This solver implements the standard incompressible Euler equations on a 2D MAC (Marker-and-Cell) grid, following the formulation in Bridson's *Fluid Simulation for Computer Graphics* (2nd ed., 2015). The fluid is treated as inviscid and constant-density; viscosity is purely numerical, arising from the advection scheme. A passive three-channel dye field (R, G, B) is advected alongside the velocity to visualize flow structure.

The project began as a learning exercise — implementing every component from first principles rather than relying on libraries — and grew into a testbed for advection schemes, pressure solvers, and stability techniques.

## Method summary

**Discretization.** Staggered MAC grid with horizontal velocity `u` stored on vertical cell faces and vertical velocity `v` stored on horizontal cell faces. Pressure, divergence, and scalar (dye) fields are stored at cell centers. Grid dimensions are `Nx × Ny`; velocity arrays are sized `(Nx+1) × Ny` and `Nx × (Ny+1)` respectively.

**Time integration.** Adaptive timestep computed from the CFL condition with a configurable safety factor (default 0.7, conservative for Shu-Osher RK3 which is unconditionally stable but better when less than CFL = 1.0 to keep backtracing within each cell).

**Pipeline per step:**

1. Compute adaptive `dt` from CFL.
2. Apply vorticity confinement as a body force (optional, configurable strength).
3. Advect velocity (semi-Lagrangian with selectable scheme, see below).
4. Enforce boundary conditions (no-slip / solid walls).
5. Compute divergence of the intermediate velocity field.
6. Solve the pressure Poisson equation (SOR, configurable over-relaxation).
7. Project velocity to be discretely divergence-free.
8. Re-enforce boundaries.
9. Advect passive scalar fields (dye).

## Implemented features

### Advection schemes

The solver implements multiple advection schemes that can be swapped in the update loop, both for the velocity field and for passive scalars:

| Scheme | Description | Theoretical order (smooth) |
| --- | --- | --- |
| **RK1 semi-Lagrangian** | First-order Euler backtrace, bilinear interpolation at the foot point. | O(h) in space, O(dt) in time |
| **RK3 semi-Lagrangian** | Shu-Osher TVD RK3 for the characteristic backtrace, bilinear interpolation. | O(h) in space, O(dt³) in time |
| **RK1 BFECC** | Back-and-Forth Error Compensation and Correction using RK1 substeps, with Kim et al. local extrema limiter (clamp to neighborhood min/max). | Approaches O(h²) in smooth regions where the limiter does not trigger |
| **RK3 BFECC** | BFECC with Shu-Osher RK3 substeps and the same local extrema limiter. | Approaches O(h²) in smooth regions where the limiter does not trigger, higher-order temporal accuracy |

BFECC (Dupont & Liu 2003; Kim et al. 2005 in graphics) performs a forward advection, a backward advection, estimates the error from the round-trip, applies a half-error correction to the initial field, and re-advects. The local extrema limiter clamps the result to the min/max of the source neighborhood to suppress new extrema and overshoots that pure BFECC can introduce.

### Pressure projection

Pressure Poisson equation solved with successive over-relaxation (SOR / Gauss-Seidel with relaxation parameter ω). The default ω = 1.95 is near-optimal for a Poisson problem on this grid size (theoretical optimum ω ≈ 2 / (1 + sin(π/N)) ≈ 1.95 for N = 128). Maximum iteration count and over-relaxation are both configurable. After the solve, velocity is projected by subtracting the pressure gradient on faces.

### Vorticity confinement

Implements the standard Fedkiw et al. 1996 vorticity confinement force to counteract numerical dissipation of vortical structures. Computes vorticity ω = ∂v/∂x − ∂u/∂y at cell centers, builds the gradient of |ω|, normalizes to get a direction N pointing toward the vortex core, and adds force `f = ε (N × ω)` interpolated to face velocities. Strength is configurable; the implementation includes guards against runaway artifacts that can appear at high ε in nearly-irrotational regions.

### Passive dye

Three independent scalar fields (R, G, B) advected with the chosen scheme. Used for flow visualization. Injection is exposed via keyboard controls.

### Boundary handling

Solid cells are marked in a per-cell `CellType` array. Boundary enforcement zeros normal velocity components on solid faces and sets tangential components to satisfy no-slip. Currently boundaries are limited to the domain walls; arbitrary solids via SDF are planned for the C++ rewrite.

### Diagnostics

The simulation exposes runtime statistics for the renderer:
- FPS
- Current `dt`
- Max `|u|`, `|v|` across the field
- Min and max divergence (for pre- and post-projection)
- Total dye mass

### Visualization (Drawer)

Rendered with [Raylib-cs](https://github.com/raylib-cs/raylib-cs). The Drawer module supports:

- Density field rendering via a `Texture2D` with per-cell RGB blending of the three dye channels
- Toggleable velocity vector field overlay, colored by local vorticity (magenta/cyan diverging palette)
- Divergence field visualization with tanh-mapped color saturation
- Per-cell index, dye, and solid-cell debug overlays (commented in source, easy to re-enable)
- Real-time statistics panel
- Frame-by-frame video export via an embedded `SimulationVideoWriter` (writes to `.mp4` directly without intermediate PNG sequences)

## Performance

Single-threaded C# implementation on CPU with RK1 BFECC and SOR. Measured on a `Ryzen 5 5600H`:

| Grid size | Cells | Approx. FPS (+- ~2) | µs/cell |
| --- | --- | --- | --- |
| 128 × 128 | 16,384 | 50 | 1.22 |
| 192 × 192 | 36,864 | 25 | 1.09 |
| 256 × 256 | 65,536 | 13 | 1.17 |
| 512 × 512 | 262,144 | 3 | 1.27 |

The pressure solve (SOR) and BFECC advection are the two dominant costs. Profiling indicates SOR is the larger of the two at default iteration counts. The C++ rewrite targets these directly via a CG solver (later multigrid) and SIMD-friendly memory layouts, with a GPU port planned for the divergence and Poisson solves.

On 128x128, the sim takes ~40 steps to get max divergence ~= 1e-3 at default SOR iterations
## Controls

| Key | Action |
| --- | --- |
| `Space` | Pause / resume simulation |
| `Enter` | Single-step (advance one frame while paused) |
| `R` | Reset simulation |
| `U` | Inject red dye (random density per cell, random size) and perturb it (randomly varying velocity magnitude towards bottom right) |
| `I` | Inject green dye (random density/size), no perturbation |
| `O` | Inject blue dye (density = 1), no perturbation |
| `P` | Perturb velocity field (thickness of 3 streaks originating at boundaries) |
| `V` | Toggle velocity vector overlay |
| `Left Shift` | Toggle slow-motion playback (10 vs 60 FPS) |
| `\` | Start / stop video recording (compiles & saves output when recording stopped) |

## Build and run

Requires .NET 8 and the Raylib-cs NuGet package.

```bash
cd legacy/csharp-eulerian
dotnet restore
dotnet run -c Release
```

## Known limitations

- **Inviscid only.** No explicit viscous stress term. Effective dissipation is numerical and depends on advection scheme and grid resolution. A FLIP/PIC blending parameter mapped to a viscosity-like coefficient is on the Iguazu roadmap (the C++ rewrite), not implemented here.
- **Solid boundaries limited to walls.** No SDF-based solids yet; planned for the rewrite.
- **`ApplyBodyForces` (gravity) is disabled** in the active update loop. The current implementation interacts poorly with boundary handling and zeros the field after roughly 50 frames once advected material reaches the lower wall. Smoke does not require gravity in any case, but this is documented as a known bug rather than a feature.
- **Pressure solve termination is by iteration count, not residual tolerance.** At ω = 1.95 the visible flow stabilizes quickly, but the true L2 residual is not driven to a fixed tolerance. Residual-versus-iteration plotting is planned but not yet wired into the visualization.
- **Minimal/rudimentary quantitative verification suite.** The solver has been validated qualitatively (recognizable vortex shedding, plausible turbulence, mass-conserving dye advection) and with divergence measurements, but lacks canonical numerical tests (Taylor-Green decay, Zalesak's disk--although this was used to debug BFECC earlier--, convergence studies). These are priority additions in the C++ rewrite.

## References

Core algorithms in this implementation follow:

- Bridson, R. *Fluid Simulation for Computer Graphics*, 2nd ed., CRC Press, 2015.
- Stam, J. "Stable Fluids." *SIGGRAPH '99*, 1999. — semi-Lagrangian advection.
- Fedkiw, R., Stam, J., Jensen, H. W. "Visual Simulation of Smoke." *SIGGRAPH '01*, 2001. — vorticity confinement formulation.
- Dupont, T. F., Liu, Y. "Back and Forth Error Compensation and Correction Methods for Removing Errors Induced by Uneven Gradients of the Level Set Function." *J. Comput. Phys.*, 2003.
- Kim, B., Liu, Y., Llamas, I., Rossignac, J. "FlowFixer: Using BFECC for Fluid Simulation." *Eurographics Workshop on Natural Phenomena*, 2005. — local extrema limiter.
- Shu, C.-W., Osher, S. "Efficient implementation of essentially non-oscillatory shock-capturing schemes." *J. Comput. Phys.*, 1988. — TVD RK3.

## Project context

This solver is the entry in the Iguazu lineage. The next-generation Iguazu codebase extends this foundation toward:

- Clean separation of solver stages behind a unified interface
- SDF-based solid boundaries and free-surface pressure projection
- PIC / FLIP / APIC particle-grid hybrids
- MLS-MPM (Hu et al. 2018) for unified fluid / sand / snow / soft-body simulation
- GPU acceleration of divergence, Poisson, and particle transfers
- A quantitative validation suite (Taylor-Green, Zalesak, dam break, rotating column, convergence studies)

See the [main Iguazu README](../../README.md) for the current state of the active codebase.

## Author

Alexander Alch, September 2025 – May 2026.
