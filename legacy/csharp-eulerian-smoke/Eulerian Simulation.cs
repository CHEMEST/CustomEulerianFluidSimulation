using Raylib_cs;
using System.Drawing;
using System.Numerics;
using System.Linq;
using CustomEulerianFluidSimulation;
using System.Diagnostics;
using System.Runtime.CompilerServices;

// World/grid coordinates: the actual coordinates of the simulation, where each cell is 1 unit.
// This is what we use for all the physics calculations since it makes more sense to have a consistent unit system for that.
// The top-left corner of the grid is (0, 0) and the bottom-right corner is (gridWidth, gridHeight).

// Screen coordinates: the pixel coordinates for drawing.
// This is what we use for rendering since Raylib works in pixels.
// The top-left corner of the screen is (0, 0) and the bottom-right corner is (screenWidth, screenHeight).
// We convert from world to screen coordinates by multiplying by cellSize, which is the number of pixels per cell.


class EulerianSimulation
{
    // Parameters
    private readonly int Nx;
    private readonly int Ny;

    private readonly int pressureIters = 50; // banding <50
    private readonly float SORterm = 1.95f; // others recommend 1.7-1.8 for optimal convergence; 1.95 converges in ~3 steps for this sim. >1.95 explodes
    private readonly float inkSize = 10f;
    private readonly int marginFactor = 8;
    private readonly Vector2 g = new Vector2(0, 9.8f); // gravity
    private readonly float maxAllowedDt = 10f;
    private readonly float vorticity = 0.1f; // strength of vorticity confinement, causes artifacts (spontanous, locked vorticies) in empty space if too high

    /// <summary>
    /// Staggered grid setup (MAC)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ref float U(int i, int j) => ref VelocityFieldX[j * Nx + i];
    public float[] VelocityFieldX { get; private set; } // I'll reference this as "U" or "u" since it's the x-velocity, but it's really the velocity on the vertical faces of the grid cells. Size (W+1, H)
    private float[] uNew; // for storing results of advection before swapping into velocityFieldX
    /// <summary>
    /// Staggered grid setup (MAC)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ref float V(int i, int j) => ref VelocityFieldY[j * Nx + i];
    public float[] VelocityFieldY { get; private set; } // I'll reference this as "V" or "v" since it's the y-velocity, but it's really the velocity on the horizontal faces of the grid cells. Size (W, H+1)
    private float[] vNew; // for storing results of advection before swapping into velocityFieldY

    private float[] inkR; // scalar per cell
    private float[] inkG; // scalar per cell
    private float[] inkB; // scalar per cell

    private float[] phiNew; // phi for all scalar fields
    private float[] divergence; // per cell
    private float[] pressure; // per cell
    private CellType[] type; // per cell
    // for vorticity confinement
    float[] omega;
    float[] mag;
    float[] vorticityMap;

    Random random = new Random();

    // Statistics | Debug
    public float dt = 0f;
    public float l2DivAfter = 0f;



    public EulerianSimulation(int width, int height)
    {
        this.Nx = width;
        this.Ny = height;
        // 2D arrays are created column, row since it's an array inside an array.
        VelocityFieldX = new float[(Nx + 1) * Ny]; // X velocities live on vertical edges, so we need an extra column (i, j+0.5)
        VelocityFieldY = new float[Nx * (Ny + 1)]; // Y velocities live on horizontal edges, so we need an extra row (i+0.5, j)
        uNew = new float[(Nx + 1) * Ny];
        vNew = new float[Nx * (Ny + 1)];

        divergence = new float[Nx * Ny];
        pressure = new float[Nx * Ny];
        inkR = new float[Nx * Ny];
        inkG = new float[Nx * Ny];
        inkB = new float[Nx * Ny];
        omega = new float[Nx * Ny];
        mag = new float[Nx * Ny];
        vorticityMap = new float[Nx * Ny];

        phiNew = new float[Nx * Ny];
        type = new CellType[Nx * Ny];


        InitializeFields();
    }
    private void InitializeFields()
    {
        for (int i = 0; i < Nx; i++)
        {
            for (int j = 0; j < Ny; j++)
            {
                divergence[j*Nx + i] = 0f;

                type[j * Nx + i] = CellType.Fluid; // u(0, j), v(i, 0), u(gridWidth, j), v(i, gridHeight) are all solid boundaries.
                //Console.Write(type[i , j] == 0 ? "SOLID | " : "");
            }
        }
        ResetSim();
    }
    public void ResetSim()
    {
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
                U(i, j) = 0f;

        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
                V(i, j) = 0f;
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                pressure[j * Nx + i] = 0f;
                inkR[j * Nx + i] = 0f;
                inkG[j * Nx + i] = 0f;
                inkB[j * Nx + i] = 0f;
            }

        ComputeDivergence();
    }
    public void InjectAndPerturbRed()
    {
        int x = (int) (random.NextDouble() * Nx) / 2;
        int y = (int) (random.NextDouble() * Ny) / 2;
        for (int i = Nx / marginFactor; i < x + Nx/marginFactor && i < Nx; i++)
        {
            for (int j = Ny / marginFactor; j < y + Ny/marginFactor && j<Ny; j++)
            {
                U(i, j) = 10 * (float)random.NextDouble();
                V(i, j) = 10 * (float) random.NextDouble();
                inkR[j * Nx + i] = (float) random.NextDouble();
            }
        }
    }
    public void InjectAndPerturbGreen()
    {
        int x = (int)(random.NextDouble() * Nx) / 2;
        int y = (int)(random.NextDouble() * Ny) / 2;
        for (int i = Nx / marginFactor; i < x + Nx / marginFactor && i < Nx; i++)
        {
            for (int j = Ny / marginFactor; j < y + Ny / marginFactor && j < Ny; j++)
            {
                //U(i, j) = 10 * (float)random.NextDouble();
                //V(i, j) = 10 * (float)random.NextDouble();

                inkG[j * Nx + i] = (float)random.NextDouble();
            }
        }
    }
    public void InjectAndPerturbBlue()
    {
        int x = Nx/3;
        int y = Ny/3;
        int shift = Nx / 3;
        for (int i = Nx / marginFactor; i < x + Nx / marginFactor&& i < Nx; i++)
        {
            for (int j = Ny / marginFactor + shift; j < y + Ny / marginFactor + shift && j < Ny; j++)
            {
                //U(i, j) = 10 * (float)random.NextDouble();
                //V(i, j) = 10 * (float)random.NextDouble();

                inkB[j * Nx + i] = 1.0f;
            }
        }
    }
    public void PerturbVelocity()
    {
        for (int r = 0; r < 4; r++)
        {
            int x = Math.Clamp((int)(random.NextDouble() * Nx), 1, Nx - 2);
            int y = Math.Clamp((int)(random.NextDouble() * Ny), 1, Ny - 2);
            for (int i = 1; i < Nx / 1.25; i++)
            {
                U(i, y+1) += 10 * (float)random.NextDouble();
                U(i, y) += 10 * (float)random.NextDouble();
                U(i, y-1) += 10 * (float)random.NextDouble();
            }
            for (int j = 1; j < Ny / 1.25; j++)
            {
                V(x-1, j) += 10 * (float)random.NextDouble();
                V(x, j) += 10 * (float)random.NextDouble();
                V(x+1, j) += 10 * (float)random.NextDouble();
            }
        }
    }
    public void InitializeRotationalField()
    {
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                U(i, j) = -(j - Ny / 2);
                V(i, j) = (i - Nx / 2);
            }
    }
    public void InitializeConstantRotationalField()
    {
        float cx = (Nx - 1) * 0.5f;
        float cy = (Ny - 1) * 0.5f;
        float speed = 1.0f;

        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                float x = (i - cx);
                float y = (j - cy);

                float r = MathF.Sqrt(x * x + y * y);

                if (r > 1e-6f)
                {
                    U(i, j) = -speed * y / r;
                    V(i, j) = speed * x / r;
                }
                else
                {
                    // center is undefined; pick zero or anything finite
                    U(i, j) = 0;
                    V(i, j) = 0;
                }
            }
    }


    public void Update(float deltaTime)
    {
        //dt = 0.5f;
        dt = CalculateTimeStep();

        //ApplyBodyForces(dt); // zeroes out the whole field after ~50 frames (Once center hits bottom wall so possibly a boundaries issue)
        VorticityConfinement(dt, vorticity); // non-physically derived force

        RK1BFECCAdvection(dt); // Bottleneck 2
        EnforceBoundaries();

        ComputeDivergence();
        ComputePoissonPressure(dt); // Bottleneck 1
        ProjectPressure(dt);
        EnforceBoundaries();

        ComputeDivergence();
        AdvectScalarFieldsRK1BFECC(dt);
    }

    private void AdvectScalarFieldsRK3BFECC(float dt)
    {
        RK3BFECCAdvectionScalar(inkR, dt);
        RK3BFECCAdvectionScalar(inkG, dt);
        RK3BFECCAdvectionScalar(inkB, dt);

    }
    private void AdvectScalarFieldsRK1BFECC(float dt)
    {
        RK1BFECCAdvectionScalar(inkR, dt);
        RK1BFECCAdvectionScalar(inkG, dt);
        RK1BFECCAdvectionScalar(inkB, dt);
    }
    private void AdvectScalarFieldsRK3(float dt)
    {
        AdvectScalarFieldRK3(inkR, dt);
        AdvectScalarFieldRK3(inkG, dt);
        AdvectScalarFieldRK3(inkB, dt);
    }

    // VC is basically just adding a force that pushes the fluid to swirl more in areas of high vorticity, which enhances the small-scale swirling motion and makes the fluid look more lively.
    // It's a common technique in fluid sims to counteract numerical dissipation and add visual interest.
    // NOTE: it is not physically derived. This is to make it more visually interesting and is not usually used in engineering CFD
    //
    // Goal: add a force that pushes the fluid to swirl more in areas of high vorticity.
    // Vorticity is the curl of the velocity field so: w = \del \cross u
    // In 2D, this simplifies to w = dv/dx - du/dy
    // Magnitude in 2D (scalar): |w| = sqrt(w^2) = abs(w)
    // Gradient of vorticity (field): grad|w| = (dw/dx, dw/dy)
    //      this points outward from the vortex center
    // Normalize: N = grad|w| / |grad|w|
    //      We do this for more control instead of following the strength of the vortex
    // Vorticity confinement force: f = k * (N x w); in 2D, N x w = (N.y * w, -N.x * w).
    //      crossing the outward and the vorticity gives a vector that points tangentially to the swirl
    //      k is a user-defined strength of the confinement (vorticity parameter) * h (cell size, which is 1 in our world coordinates) to keep it consistent across different grid resolutions.
    // Average force at cells adjacent to faces and add to velocity field as a body force since MAC makes this math easier for cell centered forces
    private void VorticityConfinement(float dt, float vorticity)
    {
        
        // Step 1, 2: compute vorticity and its magnitude at cell centers. We can skip the boundaries since we'll just set the confinement force to zero there anyway.
        for (int i = 1; i < Nx - 1; i++)
            for (int j = 1; j < Ny - 1; j++)
            {
                omega[j*Nx + i] = (V(i + 1, j) - V(i, j)) - (U(i, j + 1) - U(i, j));
                mag[j*Nx + i] = Math.Abs(omega[j*Nx + i]);
            }
        // Step 3,4,5
        for (int i = 1; i < Nx - 1 ; i++)
            for (int j = 1; j < Ny - 1 ; j++)
            {
                float dw_dx = (mag[j*Nx + (i + 1)] - mag[j*Nx + (i - 1)]) / 2f;
                float dw_dy = (mag[(j + 1)*Nx + i] - mag[(j - 1)*Nx + i]) / 2f;
                float gradMag = (float)Math.Sqrt(dw_dx * dw_dx + dw_dy * dw_dy);
                if (gradMag < 1e-1f) continue; // avoid division by zero and skip if there's no significant vorticity
                float Normx = dw_dx / gradMag;
                float Normy = dw_dy / gradMag;

                // confinement force at cell center
                float fx = vorticity * (Normy * omega[j*Nx + i]);
                float fy = vorticity * (-Normx * omega[j*Nx + i]);

                // apply half the force to each adjacent face (since the force is at the cell center and we want to distribute it to the faces)
                U(i, j) += fx * dt / 2f; // right face of cell (i, j)
                U(i + 1, j) += fx * dt / 2f; // left face of cell (i+1, j)
                V(i, j) += fy * dt / 2f; // top face of cell (i, j)
                V(i, j + 1) += fy * dt / 2f; // bottom face of cell (i, j+1)
            }
    }
    private float CalculateTimeStep()
    {
        float uMax = 0f;
        for (int i = 0; i < Nx + 1; i++)
            for (int j = 0; j < Ny; j++)
                uMax = Math.Max(uMax, Math.Abs(U(i, j)));
        float vMax = 0f;
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny + 1; j++)
                vMax = Math.Max(vMax, Math.Abs(V(i, j)));
        float maxVel = Math.Max(uMax, vMax);
        if (maxVel < 1e-6f) return maxAllowedDt;

        // Bridson derives alpha * h / maxU, but since our h = 1 in world coordinates, we can just do alpha / maxU.
        // The alpha is just a safety factor to ensure stability; tune it as needed.
        float alpha = 0.7f; // CFL safety factor; Shu-Osher RK3 is stable to 1.0, 0.5 is conservative
        return Math.Min(alpha / maxVel, maxAllowedDt);
    }
    private bool IsSolidCell(int i, int j)
    {
        // Treat outside-the-grid as solid boundary
        if (i <= 0 || i > Nx || j <= 0 || j > Ny)
            return true;

        return type[j * Nx + i] == CellType.Solid;
    }
    // Enforce no-penetration: any face adjacent to a solid cell has normal velocity = 0
    private void EnforceBoundaries()
    {
        // --- U faces: size (W+1, H), u[i,j] is face between cell (i-1,j) and (i,j)
        // 1) Hard domain boundaries: left wall u[0,*] and right wall u[W,*]
        for (int j = 0; j < Ny; j++)
        {
            U(0, j) = 0f;
            U(Nx, j) = 0f;
        }

        //// 2) Interior faces: if either adjacent cell is solid, zero it
        //for (int i = 1; i < gridWidth; i++)
        //{
        //    for (int j = 0; j < gridHeight; j++)
        //    {
        //        bool leftSolid = (type[i - 1, j] == CellType.Solid);
        //        bool rightSolid = (type[i, j] == CellType.Solid);

        //        if (leftSolid || rightSolid)
        //            velocityFieldX[i, j] = 0f;
        //    }
        //}

        // --- V faces: size (W, H+1), v[i,j] is face between cell (i,j-1) and (i,j)
        // 1) Hard domain boundaries: bottom wall v[* ,0] and top wall v[* ,H]
        for (int i = 0; i < Nx; i++)
        {
            V(i, 0) = 0f;
            V(i, Ny) = 0f;
        }

        //// 2) Interior faces: if either adjacent cell is solid, zero it
        //for (int i = 0; i < gridWidth; i++)
        //{
        //    for (int j = 1; j < gridHeight; j++)
        //    {
        //        bool bottomSolid = (type[i, j - 1] == CellType.Solid);
        //        bool topSolid = (type[i, j] == CellType.Solid);

        //        if (bottomSolid || topSolid)
        //            velocityFieldY[i, j] = 0f;
        //    }
        //}
    }

    private void ApplyBodyForces(float dt)
    {
        for (int i = 0; i < Nx; i++)
        {
            for (int j = 0; j < Ny; j++)
            {
                V(i, j) += g.Y * dt;
            }
        }
    }
    /// <summary>
    /// Laplacian of pressure = the sum of the pressures in the S neighboring faces - S * pressure in the current cell.
    /// Where S is the number of non-solid neighbors.
    /// This is the discrete version of the continuous Laplacian operator.
    /// 
    /// </summary>
    /// <param name="dt"></param>
    private void ComputePoissonPressure(float dt)
    {
        for (int n = 0; n < pressureIters; n++)
        {
            //Console.WriteLine($"Poisson iteration {n + 1}/{pressureIters}");
            for (int i = 0; i < Nx; i++)
            {
                //Console.WriteLine($"  Row {i}/{gridWidth - 1}");
                for (int j = 0; j < Ny; j++)
                {
                    //Console.WriteLine($"    Cell ({i}, {j})");
                    if (type[j * Nx + i] == CellType.Solid) { pressure[j * Nx + i] = 0; continue; } // no pressure in solids since they don't move.

                    float sum = 0;
                    int count = 0;
                    // Check 4 neighbors

                    // Left neighbor
                    if (i > 0 && type[j * Nx + (i - 1)] != CellType.Solid) { sum += pressure[j * Nx + (i - 1)]; count++; }
                    // Right neighbor
                    if (i < Nx - 1 && type[j * Nx + (i + 1)] != CellType.Solid) { sum += pressure[j * Nx + (i + 1)]; count++; }
                    // Bottom neighbor
                    if (j > 0 && type[(j - 1) * Nx + (i)] != CellType.Solid) { sum += pressure[(j - 1) * Nx + (i)]; count++; }
                    // Top neighbor
                    if (j < Ny - 1 && type[(j + 1) * Nx + (i)] != CellType.Solid) { sum += pressure[(j + 1) * Nx + (i)]; count++; }

                    if (count == 0) { pressure[j * Nx + i] = 0; continue; } // if a fluid cell is completely surrounded by solid cells for whatever reason, it shouldn't build up infinite pressure since it can't move anyway, so we can just set its pressure to 0 and move on.

                    // the "source" term for the Poisson equation is the divergence of the velocity field.
                    // We want to find the pressure field that will counteract this divergence and make the velocity field incompressible.
                    // Many sources like to say that "the pressure is exactly whatever we need to make the divergence = 0,"
                    // but that's not really right.
                    // The pressure is whatever we need to make the divergence = 0 after we apply the pressure gradient to the velocity field.
                    // The pressure itself doesn't directly set the divergence to zero;
                    // it's the pressure gradient that does that when we subtract it from the velocity field in the projection step.
                    // This is basically the Helmholtz decomposition: some vector fields (like our velocity field) can be decomposed into a divergence-free part and a curl-free part.
                    // The pressure gradient is what we subtract from the velocity field to remove the curl-free part and enforce incompressibility.

                    // the 1/dt is because the divergence is like a "velocity change per second,"
                    // and we want to find the pressure that will counteract that change over this time step.
                    // It's basically saying "how much pressure do we need to apply to counteract this divergence over the next dt seconds?"
                    float rhs = divergence[j * Nx + i] / dt;
                    //Console.WriteLine($"Divergence at ({i}, {j}): {divergence[i, j]}, RHS for Poisson: {rhs}");
                    // Neumann boundary conditions are implicitly handled here by only counting non-solid neighbors and summing their pressures.

                    // Successive Over-Relaxation (SOR): we can use the newly computed pressure values in the same iteration to speed up convergence.
                    float pNew = (sum - rhs) / count;
                    pressure[j * Nx + i] = Single.Lerp(pressure[j * Nx + i], pNew, SORterm); // Keep the over-relaxation term between 1.7 and 1.95 for optimal convergence. 1.8 is a common choice.

                }
            }
        }
    }
    /// <summary>
    /// The pressure gradient is the change in pressure across a cell, 
    /// and it represents the force that the fluid feels due to pressure differences.
    /// This takes the pressure field we computed in ComputePressure and 
    /// applies its gradient to the velocity field to enforce incompressibility (divField = 0).
    /// </summary>
    /// <param name="dt"></param>
    private void ProjectPressure(float dt)
    {

        // U faces: size (W+1, H), face between cells (i-1,j) and (i,j)
        for (int i = 1; i < Nx; i++)
        {
            for (int j = 0; j < Ny; j++)
            {
                //if (type[i, j] == CellType.Solid || type[i - 1, j] == CellType.Solid) continue; // skip faces adjacent to solids
                float gradP = pressure[j * Nx + i] - pressure[j * Nx + (i - 1)];
                U(i, j) -= gradP * dt;
            }
        }
        // V faces: size (W, H+1), face between cells (i,j-1) and (i,j)
        for (int i = 0; i < Nx; i++)
        {
            for (int j = 1; j < Ny; j++)
            {
                //if (type[i, j] == CellType.Solid || type[i, j - 1] == CellType.Solid) continue; // skip faces adjacent to solids
                float gradP = pressure[j * Nx + i] - pressure[(j - 1) * Nx + (i)];
                V(i, j) -= gradP * dt;
            }
        }
    }
    /// <summary>
    /// Divergence is the inflow/outflow of a cell.
    /// If more fluid is flowing in than out, divergence is positive.
    /// If more fluid is flowing out than in, divergence is negative.
    /// If the flow is perfectly balanced, divergence is zero. 
    /// In an incompressible fluid, we want to enforce zero divergence everywhere (except intentional sources/sinks),
    ///     which is what the pressure projection step does with this divergence value.
    ///     
    /// Discretization:
    /// divergence = du/dx + dv/dy; this is literally "velocity change across the cell in x, plus y." In projection, we'll want to say "no change in total velocity from end-to-end within a cell"
    /// du/dx = u(i+1, j) - u(i, j) since u is on vertical edges;
    /// dv/dy = v(i, j+1) - v(i, j) since v is on horizontal edges.
    /// This is a local approximation of the divergence at the center of the cell, but that's just what discretization is.
    /// </summary>
    private void ComputeDivergence()
    {
        for (int i = 0; i < Nx; i++)
        {
            for (int j = 0; j < Ny; j++)
            {
                // h = 1 since we're working in world coordinates where cell size is 1 unit,
                // so we can just do the subtraction without dividing by h.
                // We do this because working in world coords makes more sense in the computational bits; i.e. leave the screen stuff for drawing phase
                divergence[j * Nx + (i)] = (U(i + 1, j) - U(i, j) +
                        V(i, j + 1) - V(i, j)) / 1;
            }
        }
    }
    /// <summary>
    /// Advection: move velocity field according to itself (NOT "pushing itself"). Advection can (and will be) applied to fields other than the velocity of the fluid to transport other information (ink, heat, etc.).
    /// This is the non-linear part of the Navier-Stokes equations and is what makes fluid sims look cool.
    /// We use semi-Lagrangian advection: for each face, we backtrace to find where the fluid at that face came from,
    /// and sample the velocity field at that point to get the new velocity for that face.
    /// I wrote a ton more in seperate notes on advection, but the gist is that this is basically just "move the velocity field according to its own structure," and it's what creates the swirling, flowing motion of fluids.
    /// Calling it the "convective acceleration" term is accurate.
    /// 
    /// RK1: First-order Runge-kutta discretization of the path integral.
    /// Backtrace once using the velocity at the face to find where the fluid came from, 
    /// and sample the velocity field at that point to get the new velocity for that face.
    /// </summary>
    private void AdvectVelocityRK1(float dt)
    {
        // u faces: i = 0..W, j = 0..H-1
        for (int i = 1; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                // face world position from grid coordinates (I don't get this fully yet)
                float x = i;
                float y = j + 0.5f;

                // backtrace (backward Euler) to find where the fluid at (x, y) came from
                Vector2 vel = SampleMACVelocity(x, y);
                float xPrev = x - dt * vel.X;
                float yPrev = y - dt * vel.Y;

                // sample OLD u-field at that backtraced position
                // We avoid in-place updates because the velocity field is used for backtracing, and if we update it in-place,
                // we would be using some new and some old values during backtracing which would lead to contaminated/incorrect results.
                // By using a separate array for the new velocities,
                // we ensure that all backtracing is done using the original velocity field from the start of the time step and then later swapped.
                uNew[j * Nx + i] = SampleU(xPrev, yPrev);
            }

        // v faces: i = 0..W-1, j = 0..H
        for (int i = 0; i < Nx; i++)
            for (int j = 1; j <= Ny; j++)
            {
                float x = i + 0.5f;
                float y = j;
                Vector2 vel = SampleMACVelocity(x, y);
                float xPrev = x - dt * vel.X;
                float yPrev = y - dt * vel.Y;
                vNew[j * Nx + i] = SampleV(xPrev, yPrev);
            }

        // swap new velocities into the main velocity fields
        for (int i = 1; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
                U(i, j) = uNew[j * Nx + i];
        for (int i = 0; i < Nx; i++)
            for (int j = 1; j <= Ny; j++)
                V(i, j) = vNew[j * Nx + i];
    }
    /// <summary>
    /// RK3 is just 3 steps of RK1 with intermediate velocity fields to get better accuracy.
    /// It's still unconditionally stable like RK1 since it's still semi-Lagrangian, 
    /// but it reduces numerical dissipation and gives sharper results.
    /// Think of it like discretizing the path integral with 3 steps instead of one.
    /// 
    /// SSPRK3 Coefficients from TVD-RK3 (Shu, Osher, 1988): https://apps.dtic.mil/sti/tr/pdf/ADA314231.pdf
    /// k0 = u(x)
    /// k1 = u(x - dt * k0)
    /// k2 = u(x - dt * (1/4 * k0 + 1/4 * k1)
    /// Xprev = x - dt * (1/6 * k0 + 1/6 * k1 + 2/3 * k2)
    /// </summary>
    /// <param name="dt"></param>
    private void AdvectVelocityRK3(float dt)
    {
        // u faces: i = 0..W, j = 0..H-1
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                // face world position from grid coordinates
                float x = i;
                float y = j + 0.5f;

                // backtrace (backward Euler) to find where the fluid at (x, y) came from
                Vector2 k0 = SampleMACVelocity(x, y);

                // stage 1
                float x1 = x - dt * k0.X;
                float y1 = y - dt * k0.Y;
                Vector2 k1 = SampleMACVelocity(x1, y1);

                // stage 2
                float x2 = x - dt * (0.25f * k0.X + 0.25f * k1.X);
                float y2 = y - dt * (0.25f * k0.Y + 0.25f * k1.Y);
                Vector2 k2 = SampleMACVelocity(x2, y2);

                // final
                float xPrev = x - dt * ((1f / 6f) * k0.X + (1f / 6f) * k1.X + (4f / 6f) * k2.X);
                float yPrev = y - dt * ((1f / 6f) * k0.Y + (1f / 6f) * k1.Y + (4f / 6f) * k2.Y);

                // sample OLD u-field at that backtraced position
                // We avoid in-place updates because the velocity field is used for backtracing, and if we update it in-place,
                // we would be using some new and some old values during backtracing which would lead to contaminated/incorrect results.
                // By using a separate array for the new velocities,
                // we ensure that all backtracing is done using the original velocity field from the start of the time step and then later swapped.
                uNew[j * Nx + i] = SampleU(xPrev, yPrev);
            }

        // v faces: i = 0..W-1, j = 0..H
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
            {
                // face world position from grid coordinates
                float x = i + 0.5f;
                float y = j;

                // backtrace (backward Euler) to find where the fluid at (x, y) came from
                Vector2 k0 = SampleMACVelocity(x, y);
                
                // stage 1
                float x1 = x - dt * k0.X;
                float y1 = y - dt * k0.Y;
                Vector2 k1 = SampleMACVelocity(x1, y1);

                // stage 2
                float x2 = x - dt * (0.25f * k0.X + 0.25f * k1.X);
                float y2 = y - dt * (0.25f * k0.Y + 0.25f * k1.Y);
                Vector2 k2 = SampleMACVelocity(x2, y2);

                // final
                float xPrev = x - dt * ((1f / 6f) * k0.X + (1f / 6f) * k1.X + (4f / 6f) * k2.X);
                float yPrev = y - dt * ((1f / 6f) * k0.Y + (1f / 6f) * k1.Y + (4f / 6f) * k2.Y);

                vNew[j * Nx + i] = SampleV(xPrev, yPrev);
            }

        // swap new velocities into the main velocity fields
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
                U(i, j) = uNew[j * Nx + i];
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
                V(i, j) = vNew[j * Nx + i];
    }

    private void AdvectScalarFieldRK3(float[] phi, float dt)
    {
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                // face world position from grid coordinates
                float x = i + 0.5f;
                float y = j + 0.5f;

                // backtrace (backward Euler) to find where the fluid at (x, y) came from
                Vector2 k0 = SampleMACVelocity(x, y);

                // stage 1
                float x1 = x - dt * k0.X;
                float y1 = y - dt * k0.Y;
                Vector2 k1 = SampleMACVelocity(x1, y1);

                // stage 2
                float x2 = x - dt * (0.25f * k0.X + 0.25f * k1.X);
                float y2 = y - dt * (0.25f * k0.Y + 0.25f * k1.Y);
                Vector2 k2 = SampleMACVelocity(x2, y2);

                // final
                float xPrev = x - dt * ((1f / 6f) * k0.X + (1f / 6f) * k1.X + (4f / 6f) * k2.X);
                float yPrev = y - dt * ((1f / 6f) * k0.Y + (1f / 6f) * k1.Y + (4f / 6f) * k2.Y);

                phiNew[j * Nx + i] = SampleScalar(xPrev, yPrev, phi);
            }   

        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
                phi[j * Nx + i] = phiNew[j * Nx + i];

    }
    private void AdvectScalarFieldRK1(float[] phi, float dt)
    {
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                // face world position from grid coordinates
                float x = i + 0.5f;
                float y = j + 0.5f;

                // backtrace (backward Euler) to find where the fluid at (x, y) came from
                Vector2 k0 = SampleMACVelocity(x, y);
                float xPrev = x - dt * k0.X;
                float yPrev = y - dt * k0.Y;

                phiNew[j * Nx + i] = SampleScalar(xPrev, yPrev, phi);
            }

        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
                phi[j * Nx + i] = phiNew[j * Nx + i];

    }
    private float FindMinWithinKernel(int i, int j, float[] phi, int kernelSize)
    {
        float minVal = float.MaxValue;
        for (int k = Math.Max(0, i - kernelSize); k <= Math.Min(Nx-1, i + kernelSize); k++)
            for (int l = Math.Max(0, j - kernelSize); l <= Math.Min(Ny-1, j + kernelSize); l++)
            {
                minVal = Math.Min(minVal, phi[l * Nx + k]);
            }
        return minVal;
    }
    private float FindMaxWithinKernel(int i, int j, float[] phi, int kernelSize)
    {
        float maxVal = float.MinValue;
        for (int k = Math.Max(0, i - kernelSize); k <= Math.Min(Nx - 1, i + kernelSize); k++)
            for (int l = Math.Max(0, j - kernelSize); l <= Math.Min(Ny - 1, j + kernelSize); l++)
            {
                maxVal = Math.Max(maxVal, phi[l * Nx + k]);
            }
        return maxVal;
    }
    private void RK3BFECCAdvectionScalar(float[] phi, float dt)
    {
        float[] BFECCtemp = new float[Nx * (Ny)];
        // 1) Store initial field
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                BFECCtemp[j * Nx + i] = phi[j * Nx + i];
            }
        // 2) Forward step
        AdvectScalarFieldRK3(phi, dt);
        // 3) Backward step
        AdvectScalarFieldRK3(phi, -dt);
        // 4) Error estimation and correction
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                float error = BFECCtemp[j * Nx + i] - phi[j * Nx + i];
                phi[j * Nx + i] = BFECCtemp[j * Nx + i] + 0.5f * error;
            }
        // 5) Step with corrected field
        AdvectScalarFieldRK3(phi, dt);
        // 6) clamp within old range
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
                phi[j * Nx + i] = Math.Clamp(phi[j * Nx + i], FindMinWithinKernel(i, j, BFECCtemp, 1), FindMaxWithinKernel(i, j, BFECCtemp, 1));
    }
    private void RK1BFECCAdvectionScalar(float[] phi, float dt)
    {
        float[] BFECCtemp = new float[Nx * (Ny)];
        // 1) Store initial field
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                BFECCtemp[j * Nx + i] = phi[j * Nx + i];
            }
        // 2) Forward step
        AdvectScalarFieldRK1(phi, dt);
        // 3) Backward step
        AdvectScalarFieldRK1(phi, -dt);
        // 4) Error estimation and correction
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                float error = BFECCtemp[j * Nx + i] - phi[j * Nx + i];
                phi[j * Nx + i] = BFECCtemp[j * Nx + i] + 0.5f * error;
            }
        // 5) Step with corrected field
        AdvectScalarFieldRK1(phi, dt);
        // 6) clamp within old range
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j < Ny; j++)
                phi[j * Nx + i] = Math.Clamp(phi[j * Nx + i], FindMinWithinKernel(i, j, BFECCtemp, 1), FindMaxWithinKernel(i, j, BFECCtemp, 1));
    }
    /// <summary>
    /// Shu-Osher RK3, BFECC, and Kim et al. local extrema limiting
    /// </summary>
    /// <param name="dt"></param>
    private void RK3BFECCAdvection(float dt)
    {
        float[] BFECCtempX = new float[(Nx + 1) * Ny];
        float[] BFECCtempY = new float[Nx * (Ny + 1)];
        // 1) Store initial fields
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
                BFECCtempX[j * Nx + i] = U(i, j);
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
                BFECCtempY[j * Nx + i] = V(i, j);
        // 2) Forward step
        AdvectVelocityRK3(dt);
        // 3) Backward step
        AdvectVelocityRK3(-dt);
        // 4) Error estimation and correction
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                float error = BFECCtempX[j * Nx + i] - U(i, j);
                U(i, j) = BFECCtempX[j * Nx + i] + 0.5f * error;
            }
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
            {
                float error = BFECCtempY[j * Nx + i] - V(i, j);
                V(i, j) = BFECCtempY[j * Nx + i] + 0.5f * error;
            }
        // 5) Step with corrected field
        AdvectVelocityRK3(dt);
        // 6) clamp within old range
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
                U(i, j) = Math.Clamp(U(i, j), FindMinWithinKernel(i, j, BFECCtempX, 1), FindMaxWithinKernel(i, j, BFECCtempX, 1));
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
                V(i, j) = Math.Clamp(V(i, j), FindMinWithinKernel(i, j, BFECCtempY, 1), FindMaxWithinKernel(i, j, BFECCtempY, 1));

    }
    private void RK1BFECCAdvection(float dt)
    {
        float[] BFECCtempX = new float[(Nx + 1) * Ny];
        float[] BFECCtempY = new float[Nx * (Ny + 1)];
        // 1) Store initial fields
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
                BFECCtempX[j * Nx + i] = U(i, j);
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
                BFECCtempY[j * Nx + i] = V(i, j);
        // 2) Forward step
        AdvectVelocityRK1(dt);
        // 3) Backward step
        AdvectVelocityRK1(-dt);
        // 4) Error estimation and correction
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
            {
                float error = BFECCtempX[j * Nx + i] - U(i, j);
                U(i, j) = BFECCtempX[j * Nx + i] + 0.5f * error;
            }
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
            {
                float error = BFECCtempY[j * Nx + i] - V(i, j);
                V(i, j) = BFECCtempY[j * Nx + i] + 0.5f * error;
            }
        // 5) Step with corrected field
        AdvectVelocityRK1(dt);
        // 6) clamp within old range
        for (int i = 0; i <= Nx; i++)
            for (int j = 0; j < Ny; j++)
                U(i, j) = Math.Clamp(U(i, j), FindMinWithinKernel(i, j, BFECCtempX, 1), FindMaxWithinKernel(i, j, BFECCtempX, 1));
        for (int i = 0; i < Nx; i++)
            for (int j = 0; j <= Ny; j++)
                V(i, j) = Math.Clamp(V(i, j), FindMinWithinKernel(i, j, BFECCtempY, 1), FindMaxWithinKernel(i, j, BFECCtempY, 1));

    }




    // Bilinear sampling of velocity fields at an arbitrary x,y position in world coordinates
    // Needed since we have a discrete grid and almost never get a perfect cordinate in backtrace
    // See in-depth comments in SampleV.
    public Vector2 SampleMACVelocity(float x, float y)
    {
        float u = SampleU(x, y);
        float v = SampleV(x, y);
        return new Vector2(u, v);
    }
    // Same as SampleV but for the U grid. See more in-depth comments in SampleV
    private float SampleU(float x, float y)
    {
        // clamp position into valid u sampling domain
        x = Math.Clamp(x, 0.0f, Nx - float.Epsilon);          // so i0 in [0..W-1], i1 = i0+1 in [1..W]
        y = Math.Clamp(y, 0.5f, Ny - 0.5f - float.Epsilon);  // so yu in [0..H-1-eps]

        // shift into u-grid coordinates: u lives at (i, j+0.5)
        float yu = y - 0.5f;

        int i0 = (int)MathF.Floor(x);
        int j0 = (int)MathF.Floor(yu);

        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp (size of U = gridWidth+1, gridHeight)
        i0 = Math.Clamp(i0, 0, Nx);
        i1 = Math.Clamp(i1, 0, Nx);
        j0 = Math.Clamp(j0, 0, Ny - 1);
        j1 = Math.Clamp(j1, 0, Ny - 1);

        float sx = x - i0;
        float sy = yu - j0;

        float v00 = U(i0, j0);
        float v10 = U(i1, j0);
        float v01 = U(i0, j1);
        float v11 = U(i1, j1);

        float vx0 = v00 + sx * (v10 - v00);
        float vx1 = v01 + sx * (v11 - v01);
        return vx0 + sy * (vx1 - vx0);
    }
    /// <summary>
    /// Bilinear sampling of the V grid:
    /// Find the four nearest known velocity values,
    /// Calculate distance of the desired point from the known points,
    /// Linear interp left-right and top-bottom,
    /// Interp those results to get the final velocity value at the desired point.
    /// 
    /// Inputs are world coords. This is important since the velocity field is defined in world coordinates 
    /// and the backtracing gives us world coordinates. 
    /// </summary>
    /// <param name="i"></param>
    /// World coordinates
    /// <param name="j"></param>
    /// World coordinates
    /// <returns></returns>
    private float SampleV(float i, float j)
    {
        // clamp position into valid v sampling domain (we do this because backtracing often requires sampling from negative or out-of-bound values)
        i = Math.Clamp(i, 0.5f, Nx - 0.5f - float.Epsilon); // so xv in [0..W-1-eps]
        j = Math.Clamp(j, 0.0f, Ny - float.Epsilon);      // so j0 in [0..H-1], j1 in [1..H]

        // shift into v-grid coordinates: v lives at (i+0.5, j)
        float xv = i - 0.5f;

        // grab the four surrounding grid points to interp from/to
        int i0 = (int)MathF.Floor(xv);
        int j0 = (int)MathF.Floor(j);
        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp within bounds (size of V = gridWidth, gridHeight+1)
        i0 = Math.Clamp(i0, 0, Nx - 1);
        i1 = Math.Clamp(i1, 0, Nx - 1);
        j0 = Math.Clamp(j0, 0, Ny);
        j1 = Math.Clamp(j1, 0, Ny);

        // distance from the lower grid point (i0, j0) to the sample position (x, y). Used for the t in interp
        float sx = xv - i0;
        float sy = j - j0;

        // sample the known velocities at the 4 surrounding points
        float v00 = V(i0, j0);
        float v10 = V(i1, j0);
        float v01 = V(i0, j1);
        float v11 = V(i1, j1);

        // two linear interps: Vf = Vi + t * (dV); top and bottom rows of the surrounding grid square
        float vy0 = v00 + sx * (v10 - v00);
        float vy1 = v01 + sx * (v11 - v01);

        // final vertical lerp between the two horizontal lerps
        return vy0 + sy * (vy1 - vy0);
    }
    // Bilinear sampling for dye and other scalar fields. Same as advection's sampling, just at the cell center instead of a face
    private float SampleScalar(float i, float j, float[] phi)
    {
        // clamp position into valid v sampling domain (we do this because backtracing often requires sampling from negative or out-of-bound values)
        i = Math.Clamp(i, 0.5f, Nx - 0.5f - float.Epsilon);
        j = Math.Clamp(j, 0.5f, Ny - 0.5f - float.Epsilon);

        float xv = i - 0.5f;
        float yv = j - 0.5f;

        // grab the four surrounding grid points to interp from/to
        int i0 = (int)MathF.Floor(xv);
        int j0 = (int)MathF.Floor(yv);
        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp within bounds
        i0 = Math.Clamp(i0, 0, Nx - 1);
        i1 = Math.Clamp(i1, 0, Nx - 1);
        j0 = Math.Clamp(j0, 0, Ny - 1);
        j1 = Math.Clamp(j1, 0, Ny - 1);

        // distance from the lower grid point (i0, j0) to the sample position (x, y). Used for the t in interp
        float sx = xv - i0;
        float sy = yv - j0;

        // sample the known velocities at the 4 surrounding points
        float v00 = phi[j0 * Nx + i0];
        float v10 = phi[j0 * Nx + i1];
        float v01 = phi[j1 * Nx + i0];
        float v11 = phi[j1 * Nx + i1];

        float vy0 = v00 + sx * (v10 - v00);
        float vy1 = v01 + sx * (v11 - v01);

        return vy0 + sy * (vy1 - vy0);
    }
    //
    public float Curl(float x, float y)
    {
        // curl = dv/dx - du/dy
        // dv/dx = v(i+1,j) - v(i,j) since v is on horizontal edges
        // du/dy = u(i,j+1) - u(i,j) since u is on vertical edges
        // for curl, we want to sample the velocity field at the cell center (i+0.5, j+0.5), so we need to shift our sampling coordinates accordingly
        float uCenter = SampleU(x, y);
        //float uRight = SampleU(x + 0.5f, y);
        float uUp = SampleU(x, y - 0.5f);
        float vCenter = SampleV(x, y);
        float vRight = SampleV(x + 0.5f, y);
        //float vUp = SampleV(x, y - 0.5f);
        float dv_dx = vRight - vCenter;
        float du_dy = uUp - uCenter;
        return dv_dx - du_dy;
    }



    private Vector2 ClampPositionToDomain(int x, int y)
    {
        return new Vector2(
            Math.Clamp(x, 0, Nx - 1),
            Math.Clamp(y, 0, Ny - 1));
    }


    // #######
    // DRAWING
    // #######
    public SimDrawData GetSimDrawData()
    {
        float minSpeed = float.MaxValue;
        float maxSpeed = float.MinValue;
        float minDiv = float.MaxValue;
        float maxDiv = float.MinValue;
        float totalDye = 0f;
        int wx = 0;
        int wy = 0;


        for (int x = 0; x < Nx; x++)
            for (int y = 0; y < Ny; y++)
            {
                float d = divergence[y * Nx + x];
                if (d < minDiv) minDiv = d;
                if (d > maxDiv) maxDiv = d;

                Vector2 vel = new Vector2(U(x, y), V(x, y)); // technically is not searching every face since it is MAC
                float speed = vel.Length();
                if (speed < minSpeed) { minSpeed = speed;  }
                    if (speed > maxSpeed) { maxSpeed = speed; wx = x; wy = y; }


                totalDye += inkR[y * Nx + x] + inkG[y * Nx + x] + inkB[y * Nx + x];
            }
        //Console.WriteLine("MaxSpeed at " + wx + "|" + wy);

        return new SimDrawData
        {
            TotalDye = totalDye,
            MinDivergence = minDiv,
            MaxDivergence = maxDiv,
            MaxSpeed = maxSpeed,
            MinSpeed = minSpeed,
            Dt = dt,
            InkR = inkR,
            InkB = inkB,
            InkG = inkG
        };
    }
    public CellDrawData[,] GetCellDrawData()
    {
        CellDrawData[,] drawData = new CellDrawData[Nx, Ny];
        for (int x = 0; x < Nx; x++)
        {
            for (int y = 0; y < Ny; y++)
            {
                drawData[x, y] = new CellDrawData
                {
                    Divergence = divergence[y * Nx + x],
                    Vorticity = omega[y * Nx + x],
                    Type = type[y * Nx + x],
                    CellVelocity = SampleMACVelocity(x + 0.5f, y + 0.5f),
                    Position = new Vector2(x, y),
                    Ink = new Vector3(inkR[y * Nx + x], inkG[y * Nx + x], inkB[y * Nx + x]),
                };
            }
        }
        return drawData;
    }

    


}