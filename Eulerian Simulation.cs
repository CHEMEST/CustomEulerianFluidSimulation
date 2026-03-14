using Raylib_cs;
using System.Drawing;
using System.Numerics;
using System.Linq;
using CustomEulerianFluidSimulation;
using System.Diagnostics;

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
    private readonly int gridWidth;
    private readonly int gridHeight;
    private readonly int pressureIters = 45; // 45-50 seems to be just barely enough
    /// <summary>
    /// Staggered grid setup (MAC)
    /// </summary>
    public float[,] VelocityFieldX { get; private set; } // I'll reference this as "U" or "u" since it's the x-velocity, but it's really the velocity on the vertical faces of the grid cells. Size (W+1, H)
    private float[,] uNew; // for storing results of advection before swapping into velocityFieldX
    /// <summary>
    /// Staggered grid setup (MAC)
    /// </summary>
    public float[,] VelocityFieldY { get; private set; } // I'll reference this as "V" or "v" since it's the y-velocity, but it's really the velocity on the horizontal faces of the grid cells. Size (W, H+1)
    private float[,] vNew; // for storing results of advection before swapping into velocityFieldY

    private float[,] dye; // scalar per cell
    private float[,] phiNew; // phi for all scalar fields
    private float[,] divergence; // per cell
    private float[,] pressure; // per cell
    private CellType[,] type; // per cell

    // Simulation Constants
    Random random = new Random();
    private readonly Vector2 g = new Vector2(0, 9.81f); // gravity

    // 0: position ; 1,2: velocity (x, y)
    Vector3[,] backtracedDataX;
    Vector3[,] backtracedDataY;

    // Statistics | Debug
    public float dt = 0f;
    public float l2DivAfter = 0f;
    public float totalDyeMass = 0f;



    public EulerianSimulation(int width, int height)
    {
        this.gridWidth = width;
        this.gridHeight = height;
        // 2D arrays are created column, row since it's an array inside an array.
        VelocityFieldX = new float[gridWidth + 1, gridHeight]; // X velocities live on vertical edges, so we need an extra column (i, j+0.5)
        VelocityFieldY = new float[gridWidth, gridHeight + 1]; // Y velocities live on horizontal edges, so we need an extra row (i+0.5, j)
        uNew = new float[gridWidth + 1, gridHeight];
        vNew = new float[gridWidth, gridHeight + 1];

        divergence = new float[gridWidth, gridHeight];
        pressure = new float[gridWidth, gridHeight];
        dye = new float[gridWidth, gridHeight];
        phiNew = new float[gridWidth, gridHeight];
        type = new CellType[gridWidth, gridHeight];

        backtracedDataX = new Vector3[gridWidth + 1, gridHeight];
        backtracedDataY = new Vector3[gridWidth, gridHeight + 1];

        InitializeFields();
    }
    private void InitializeFields()
    {
        for (int i = 0; i < gridWidth; i++)
        {
            for (int j = 0; j < gridHeight; j++)
            {
                divergence[i, j] = 0f;
                backtracedDataX[i, j] = Vector3.Zero;
                backtracedDataY[i, j] = Vector3.Zero;
                dye[i, j] = (i > 2 && i < 5 && j > 2 && j < 6) ? 1f : 0f;

                type[i, j] = CellType.Fluid; // u(0, j), v(i, 0), u(gridWidth, j), v(i, gridHeight) are all solid boundaries.
                //Console.Write(type[i , j] == 0 ? "SOLID | " : "");
            }
        }
        RandomizeVelocities();

    }
    public void RandomizeVelocities()
    {
        float scale = 10f;
        for (int i = 0; i <= gridWidth; i++)
            for (int j = 0; j < gridHeight; j++)
                VelocityFieldX[i, j] = ((float)random.NextDouble()*2 - 1) * scale;

        for (int i = 0; i < gridWidth; i++)
            for (int j = 0; j <= gridHeight; j++)
                VelocityFieldY[i, j] = ((float)random.NextDouble() * 2 - 1) * scale;

        ComputeDivergence();
    }
    public void Update(float deltaTime)
    {
        dt = 0.01f;
        //dt = CalculateTimeStep();

        //ApplyBodyForces(dt);
        EnforceBoundaries();


        //// Does advection need to run on a divergence free field?
        ComputeDivergence();
        ComputePoissonPressure(dt);
        ProjectPressure(dt);
        EnforceBoundaries();

        //AdvectVelocityRK3(dt); //huge dissipation for some reason, maybe a bug in the backtracing or sampling?
        EnforceBoundaries();

        ComputeDivergence();
        ComputePoissonPressure(dt);
        ProjectPressure(dt);

        EnforceBoundaries();
        ComputeDivergence();

        AdvectScalarFieldRK3(dye);
    }

    private float CalculateTimeStep()
    {
        float uMax = 0f;
        for (int i = 0; i < gridWidth + 1; i++)
            for (int j = 0; j < gridHeight; j++)
                uMax = Math.Max(uMax, Math.Abs(VelocityFieldX[i, j]));
        float vMax = 0f;
        for (int i = 0; i < gridWidth; i++)
            for (int j = 0; j < gridHeight + 1; j++)
                vMax = Math.Max(vMax, Math.Abs(VelocityFieldY[i, j]));
        // Bridson derives CFL * h / maxU, but since our h = 1 in world coordinates, we can just do k / maxU.
        // The 1 is just a safety factor to ensure stability; tune it as needed.
        return 1f / Math.Max(uMax, vMax);
    }
    private bool IsSolidCell(int i, int j)
    {
        // Treat outside-the-grid as solid boundary
        if (i <= 0 || i > gridWidth || j <= 0 || j > gridHeight)
            return true;

        return type[i, j] == CellType.Solid;
    }
    // Enforce no-penetration: any face adjacent to a solid cell has normal velocity = 0
    private void EnforceBoundaries()
    {
        // --- U faces: size (W+1, H), u[i,j] is face between cell (i-1,j) and (i,j)
        // 1) Hard domain boundaries: left wall u[0,*] and right wall u[W,*]
        for (int j = 0; j < gridHeight; j++)
        {
            VelocityFieldX[0, j] = 0f;
            VelocityFieldX[gridWidth, j] = 0f;
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
        for (int i = 0; i < gridWidth; i++)
        {
            VelocityFieldY[i, 0] = 0f;
            VelocityFieldY[i, gridHeight] = 0f;
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
        for (int i = 0; i < gridWidth; i++)
        {
            for (int j = 0; j < gridHeight; j++)
            {
                VelocityFieldY[i, j] += g.Y * dt;
            }
        }
    }
    /// <summary>
    /// Laplacian of pressure = the sum of the pressures in the S neighboring cells - S * pressure in the current cell.
    /// Where S is the number of non-solid neighbors.
    /// This is the discrete version of the continuous Laplacian operator.
    /// </summary>
    /// <param name="dt"></param>
    private void ComputePoissonPressure(float dt)
    {
        for (int n = 0; n < pressureIters; n++)
        {
            //Console.WriteLine($"Poisson iteration {n + 1}/{pressureIters}");
            for (int i = 1; i < gridWidth-1; i++)
            {
                //Console.WriteLine($"  Row {i}/{gridWidth - 1}");
                for (int j = 1; j < gridHeight-1; j++)
                {
                    //Console.WriteLine($"    Cell ({i}, {j})");
                    if (type[i, j] == CellType.Solid) { pressure[i, j] = 0; continue; } // no pressure in solids since they don't move.

                    float sum = 0;
                    int count = 0;
                    // Check 4 neighbors
                    if (type[i - 1, j] != CellType.Solid) { sum += pressure[i - 1, j]; count++; }
                    if (type[i + 1, j] != CellType.Solid) { sum += pressure[i + 1, j]; count++; }
                    if (type[i, j - 1] != CellType.Solid) { sum += pressure[i, j - 1]; count++; }
                    if (type[i, j + 1] != CellType.Solid) { sum += pressure[i, j + 1]; count++; }
                    if (count == 0) { pressure[i, j] = 0; continue; } // if a fluid cell is completely surrounded by solid cells for whatever reason, it shouldn't build up infinite pressure since it can't move anyway, so we can just set its pressure to 0 and move on.

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
                    float rhs = divergence[i, j] / dt;
                    //Console.WriteLine($"Divergence at ({i}, {j}): {divergence[i, j]}, RHS for Poisson: {rhs}");
                    // Neumann boundary conditions are implicitly handled here by only counting non-solid neighbors and summing their pressures.

                    // Successive Over-Relaxation (SOR): we can use the newly computed pressure values in the same iteration to speed up convergence.
                    float pNew = (sum - rhs) / count;
                    pressure[i, j] = Single.Lerp(pressure[i, j], pNew, 1.8f); // Keep the over-relaxation term between 1.7 and 1.95 for optimal convergence. 1.8 is a common choice.

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
        for (int i = 1; i < gridWidth; i++)
        {
            for (int j = 0; j < gridHeight; j++)
            {
                //if (type[i, j] == CellType.Solid || type[i - 1, j] == CellType.Solid) continue; // skip faces adjacent to solids
                float gradP = pressure[i, j] - pressure[i - 1, j];
                VelocityFieldX[i, j] -= gradP * dt;
            }
        }
        // V faces: size (W, H+1), face between cells (i,j-1) and (i,j)
        for (int i = 0; i < gridWidth; i++)
        {
            for (int j = 1; j < gridHeight; j++)
            {
                //if (type[i, j] == CellType.Solid || type[i, j - 1] == CellType.Solid) continue; // skip faces adjacent to solids
                float gradP = pressure[i, j] - pressure[i, j - 1];
                VelocityFieldY[i, j] -= gradP * dt;
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
        for (int i = 0; i < gridWidth; i++)
        {
            for (int j = 0; j < gridHeight; j++)
            {
                // h = 1 since we're working in world coordinates where cell size is 1 unit,
                // so we can just do the subtraction without dividing by h.
                // We do this because working in world coords makes more sense in the computational bits; i.e. leave the screen stuff for drawing phase
                divergence[i, j] = (VelocityFieldX[i + 1, j] - VelocityFieldX[i, j] +
                        VelocityFieldY[i, j + 1] - VelocityFieldY[i, j]) / 1;

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
        for (int i = 1; i <= gridWidth; i++)
            for (int j = 0; j < gridHeight; j++)
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
                uNew[i, j] = SampleU(xPrev, yPrev);
            }

        // v faces: i = 0..W-1, j = 0..H
        for (int i = 0; i < gridWidth; i++)
            for (int j = 1; j <= gridHeight; j++)
            {
                float x = i + 0.5f;
                float y = j;
                Vector2 vel = SampleMACVelocity(x, y);
                float xPrev = x - dt * vel.X;
                float yPrev = y - dt * vel.Y;
                vNew[i, j] = SampleV(xPrev, yPrev);
            }

        // swap new velocities into the main velocity fields
        for (int i = 1; i <= gridWidth; i++)
            for (int j = 0; j < gridHeight; j++)
                VelocityFieldX[i, j] = uNew[i, j];
        for (int i = 0; i < gridWidth; i++)
            for (int j = 1; j <= gridHeight; j++)
                VelocityFieldY[i, j] = vNew[i, j];
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
        for (int i = 1; i <= gridWidth; i++)
            for (int j = 0; j < gridHeight; j++)
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
                Vector2 vel2 = SampleMACVelocity(x2, y2);

                // final
                float xPrev = x - dt * ((1f / 6f) * k0.X + (1f / 6f) * k1.X + (4f / 6f) * vel2.X);
                float yPrev = y - dt * ((1f / 6f) * k0.Y + (1f / 6f) * k1.Y + (4f / 6f) * vel2.Y);

                // sample OLD u-field at that backtraced position
                // We avoid in-place updates because the velocity field is used for backtracing, and if we update it in-place,
                // we would be using some new and some old values during backtracing which would lead to contaminated/incorrect results.
                // By using a separate array for the new velocities,
                // we ensure that all backtracing is done using the original velocity field from the start of the time step and then later swapped.
                uNew[i, j] = SampleU(xPrev, yPrev);
            }

        // v faces: i = 0..W-1, j = 0..H
        for (int i = 0; i < gridWidth; i++)
            for (int j = 1; j <= gridHeight; j++)
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
                Vector2 vel2 = SampleMACVelocity(x2, y2);

                // final
                float xPrev = x - dt * ((1f / 6f) * k0.X + (1f / 6f) * k1.X + (4f / 6f) * vel2.X);
                float yPrev = y - dt * ((1f / 6f) * k0.Y + (1f / 6f) * k1.Y + (4f / 6f) * vel2.Y);

                vNew[i, j] = SampleV(xPrev, yPrev);
            }

        // swap new velocities into the main velocity fields
        for (int i = 1; i <= gridWidth; i++)
            for (int j = 0; j < gridHeight; j++)
                VelocityFieldX[i, j] = uNew[i, j];
        for (int i = 0; i < gridWidth; i++)
            for (int j = 1; j <= gridHeight; j++)
                VelocityFieldY[i, j] = vNew[i, j];
    }

    private void AdvectScalarFieldRK3(float[,] field)
    {
        for (int i = 0; i < gridWidth; i++)
            for (int j = 0; j < gridHeight; j++)
            {
                // face world position from grid coordinates
                float x = i + 0.5f;
                float y = j + 0.5f;

                // backtrace (backward Euler) to find where the fluid at (x, y) came from
                float k0 = SampleScalar(x, y);

                // stage 1
                float x1 = x - dt * k0;
                float y1 = y - dt * k0;
                float k1 = SampleScalar(x1, y1);

                // stage 2
                float x2 = x - dt * (0.25f * k0 + 0.25f * k1);
                float y2 = y - dt * (0.25f * k0 + 0.25f * k1);
                float vel2 = SampleScalar(x2, y2);

                // final
                float xPrev = x - dt * ((1f / 6f) * k0 + (1f / 6f) * k1 + (4f / 6f) * vel2);
                float yPrev = y - dt * ((1f / 6f) * k0 + (1f / 6f) * k1 + (4f / 6f) * vel2);

                phiNew[i, j] = SampleScalar(xPrev, yPrev);
            }

        for (int i = 0; i < gridWidth; i++)
            for (int j = 0; j < gridHeight; j++)
                dye[i, j] = phiNew[i, j];

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
        x = Math.Clamp(x, 0.0f, gridWidth - float.Epsilon);          // so i0 in [0..W-1], i1 = i0+1 in [1..W]
        y = Math.Clamp(y, 0.5f, gridHeight - 0.5f - float.Epsilon);  // so yu in [0..H-1-eps]

        // shift into u-grid coordinates: u lives at (i, j+0.5)
        float yu = y - 0.5f;

        int i0 = (int)MathF.Floor(x);
        int j0 = (int)MathF.Floor(yu);

        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp (size of U = gridWidth+1, gridHeight)
        i0 = Math.Clamp(i0, 0, gridWidth);
        i1 = Math.Clamp(i1, 0, gridWidth);
        j0 = Math.Clamp(j0, 0, gridHeight - 1);
        j1 = Math.Clamp(j1, 0, gridHeight - 1);

        float sx = x - i0;
        float sy = yu - j0;

        float v00 = VelocityFieldX[i0, j0];
        float v10 = VelocityFieldX[i1, j0];
        float v01 = VelocityFieldX[i0, j1];
        float v11 = VelocityFieldX[i1, j1];

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
        i = Math.Clamp(i, 0.5f, gridWidth - 0.5f - float.Epsilon); // so xv in [0..W-1-eps]
        j = Math.Clamp(j, 0.0f, gridHeight - float.Epsilon);      // so j0 in [0..H-1], j1 in [1..H]

        // shift into v-grid coordinates: v lives at (i+0.5, j)
        float xv = i - 0.5f;

        // grab the four surrounding grid points to interp from/to
        int i0 = (int)MathF.Floor(xv);
        int j0 = (int)MathF.Floor(j);
        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp within bounds (size of V = gridWidth, gridHeight+1)
        i0 = Math.Clamp(i0, 0, gridWidth - 1);
        i1 = Math.Clamp(i1, 0, gridWidth - 1);
        j0 = Math.Clamp(j0, 0, gridHeight);
        j1 = Math.Clamp(j1, 0, gridHeight);

        // distance from the lower grid point (i0, j0) to the sample position (x, y). Used for the t in interp
        float sx = xv - i0;
        float sy = j - j0;

        // sample the known velocities at the 4 surrounding points
        float v00 = VelocityFieldY[i0, j0];
        float v10 = VelocityFieldY[i1, j0];
        float v01 = VelocityFieldY[i0, j1];
        float v11 = VelocityFieldY[i1, j1];

        // two linear interps: Vf = Vi + t * (dV); top and bottom rows of the surrounding grid square
        float vy0 = v00 + sx * (v10 - v00);
        float vy1 = v01 + sx * (v11 - v01);

        // final vertical lerp between the two horizontal lerps
        return vy0 + sy * (vy1 - vy0);
    }
    // Bilinear sampling for dye and other scalar fields. Same as advection's sampling, just at the cell center instead of a face
    private float SampleScalar(float i, float j)
    {
        // clamp position into valid v sampling domain (we do this because backtracing often requires sampling from negative or out-of-bound values)
        i = Math.Clamp(i, 0.5f, gridWidth - 0.5f - float.Epsilon); // so xv in [0..W-1-eps]
        j = Math.Clamp(j, 0.0f, gridHeight - 0.5f - float.Epsilon);      // so j0 in [0..H-1], j1 in [1..H]

        float xv = i - 0.5f;
        float yv = j - 0.5f;

        // grab the four surrounding grid points to interp from/to
        int i0 = (int)MathF.Floor(xv);
        int j0 = (int)MathF.Floor(yv);
        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp within bounds
        i0 = Math.Clamp(i0, 0, gridWidth - 1);
        i1 = Math.Clamp(i1, 0, gridWidth - 1);
        j0 = Math.Clamp(j0, 0, gridHeight - 1);
        j1 = Math.Clamp(j1, 0, gridHeight - 1);

        // distance from the lower grid point (i0, j0) to the sample position (x, y). Used for the t in interp
        float sx = xv - i0;
        float sy = yv - j0;

        // sample the known velocities at the 4 surrounding points
        float v00 = VelocityFieldY[i0, j0];
        float v10 = VelocityFieldY[i1, j0];
        float v01 = VelocityFieldY[i0, j1];
        float v11 = VelocityFieldY[i1, j1];

        float vy0 = v00 + sx * (v10 - v00);
        float vy1 = v01 + sx * (v11 - v01);

        return vy0 + sy * (vy1 - vy0);
    }
    private Vector2 ClampPositionToDomain(int x, int y)
    {
        return new Vector2(
            Math.Clamp(x, 0, gridWidth - 1),
            Math.Clamp(y, 0, gridHeight - 1));
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


        for (int x = 0; x < gridWidth; x++)
            for (int y = 0; y < gridHeight; y++)
            {
                float d = divergence[x, y];
                if (d < minDiv) minDiv = d;
                if (d > maxDiv) maxDiv = d;

                Vector2 vel = new Vector2(VelocityFieldX[x, y], VelocityFieldY[x, y]); // technically is not searching every face since it is MAC
                float u = Math.Abs(vel.X);
                float v = Math.Abs(vel.Y);
                if (u < minSpeed) minSpeed = u;
                if (v < minSpeed) minSpeed = v;
                if (u > maxSpeed) maxSpeed = u;
                if (v > maxSpeed) maxSpeed = v;
            }


        return new SimDrawData
        {
            MinDivergence = minDiv,
            MaxDivergence = maxDiv,
            MaxSpeed = maxSpeed,
            MinSpeed = minSpeed,
            dt = dt,
        };
    }
    public CellDrawData[,] GetCellDrawData()
    {
        CellDrawData[,] drawData = new CellDrawData[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                drawData[x, y] = new CellDrawData
                {
                    Divergence = divergence[x, y],
                    Type = type[x, y],
                    CellVelocity = SampleMACVelocity(x + 0.5f, y + 0.5f),
                    Position = new Vector2(x, y),
                    Dye = dye[x, y]
                };
            }
        }
        return drawData;
    }

    


}