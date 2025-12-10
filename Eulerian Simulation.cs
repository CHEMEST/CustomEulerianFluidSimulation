using Raylib_cs;
using System.Drawing;
using System.Numerics;
using System.Linq;

class EulerianSimulation
{
    // Parameters
    private readonly int gridWidth;
    private readonly int gridHeight;
    private readonly int pressureIters = 60;
    /// <summary>
    /// cell size in pixels
    /// </summary>
    private readonly float cellSize;
    /// <summary>
    /// Staggered grid setup (MAC)
    /// </summary>
    private float[,] velocityFieldX;
    /// <summary>
    /// Staggered grid setup (MAC)
    /// </summary>
    private float[,] velocityFieldY;
    private float[,] divergence; // per cell
    private float[,] pressure; // per cell
    private int[,] type; // per cell
    private Vector2[] bodyForces = new Vector2[1]; // not in use

    // Simulation Constants
    Random random = new Random();
    private readonly Vector2 g = new Vector2(0, 9.81f); // gravity

    // 0: position ; 1,2: velocity (x, y)
    Vector3[,] backtracedDataX;
    Vector3[,] backtracedDataY;

    // Statistics | Debug
    public float dt = 0f;
    public float maxSpeed = 0f;
    public float minDiv = 0f;
    public float maxDiv = 0f;
    public float l2DivAfter = 0f;
    public float totalDyeMass = 0f;

    public EulerianSimulation(int width, int height, float cellSize)
    {
        this.gridWidth = width;
        this.gridHeight = height;
        this.cellSize = cellSize;
        velocityFieldX = new float[gridWidth + 1, gridHeight];
        velocityFieldY = new float[gridWidth, gridHeight + 1];
        divergence = new float[gridWidth, gridHeight];
        type = new int[gridWidth, gridHeight];

        backtracedDataX = new Vector3[gridWidth + 1, gridHeight];
        backtracedDataY = new Vector3[gridWidth, gridHeight + 1];

        InitializeFields();

        bodyForces[0] = g;
    }
    private void InitializeFields()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                divergence[x, y] = 0f;
                backtracedDataX[x, y] = Vector3.Zero;
                backtracedDataY[x, y] = Vector3.Zero;
                type[x, y] = (x == 0 || x == gridWidth - 1 || y == 0 || y == gridHeight - 1) ? 0 : 1; // borders are 0, inside is 1

            }
        }
        RandomizeVelocities();

    }
    public void RandomizeVelocities()
    {
        float scale = 10f;
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                velocityFieldX[x, y] = ((float)random.NextDouble()*2 - 1) * scale;
                velocityFieldY[x, y] = ((float)random.NextDouble()*2 - 1) * scale;
            }
        }
        ComputeDivergence();
    }
    public void Update(float deltaTime)
    {
        //CalculateCFLCondition
        //ApplyBodyForces(deltaTime);
        //Diffuse() | for viscous later
        // boundaries
        ComputeDivergence();
        SolvePoissonPressure(deltaTime);
        AdvectVelocity(deltaTime);

        //ProjectPressure(deltaTime);
        // apply boundaries

    }

    private void ApplyBodyForces(float dt)
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                velocityFieldY[x, y] += g.Y * dt;
            }
        }
    }

    private void SolvePoissonPressure(float dt)
    {
        for (int n = 0; n < pressureIters; n++)
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    if (type[x, y] == 0) continue;
                    float div = divergence[x, y];
                    float s = type[x + 1, y] + type[x - 1, y] + type[x, y + 1] + type[x, y - 1];
                    float o = 1.9f; //overrelaxation

                    velocityFieldX[x, y] += o * dt * (div * type[x - 1, y] / s);
                    velocityFieldX[x + 1, y] += o * dt * (-div * type[x + 1, y] / s);
                    velocityFieldY[x, y] += o * dt * (div * type[x, y - 1] / s);
                    velocityFieldY[x, y + 1] += o * dt * (-div * type[x, y + 1] / s);
                }
            }
    }
    // currently does NOT handle boundaries. Assumes infinite fluid (fix later by generalizing the divergence computation and Poisson solver)
    private void ProjectPressure(float dt)
    {
        throw new NotImplementedException();
    }

    private void ComputeDivergence()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                divergence[x, y] = (velocityFieldX[x + 1, y] - velocityFieldX[x, y] +
                        velocityFieldY[x, y + 1] - velocityFieldY[x, y]);

            }
        }
    }

    private void AdvectVelocity(float dt)
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                // Backtrace to find old position | x,y are indicies/world cords, not screen cords
                if (type[x, y] == 0) continue;
                // center of the cell in world coordinates, not screen.
                float oldPosX = x + 0.5f - velocityFieldX[x, y] * dt / cellSize;
                float oldPosY = y + 0.5f - velocityFieldY[x, y] * dt / cellSize;

                Vector2 oldVel = SampleMACVelocity(oldPosX, oldPosY);
                backtracedDataX[x, y] = new Vector3(oldPosX, oldVel.X, oldVel.Y);
                backtracedDataY[x, y] = new Vector3(oldPosY, oldVel.X, oldVel.Y);

                velocityFieldX[x, y] = oldVel.X;
                velocityFieldY[x, y] = oldVel.Y;
            }
        }
    }


    // Bilinear sampling of velocity fields at any arbitrary x,y position
    // Needed since we have a discrete grid and almost never get a perfect cordinate in backtrace
    // input is world coordinates
    // Actively working on derivation (I get it but I need to load in a bunch of info each time I try and explain it so I want a stronger interpretation)
    public Vector2 SampleMACVelocity(float x, float y)
    {
        float u = SampleU(x, y);
        float v = SampleV(x, y);
        return new Vector2(u, v);
    }
    private float SampleU(float x, float y)
    {
        // shift into u-grid coordinates: u lives at (i, j+0.5)
        float yu = y - 0.5f;

        int i0 = (int)MathF.Floor(x);
        int j0 = (int)MathF.Floor(yu);

        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp (sizeU = gridWidth+1, gridHeight)
        i0 = Math.Clamp(i0, 0, gridWidth);
        i1 = Math.Clamp(i1, 0, gridWidth);
        j0 = Math.Clamp(j0, 0, gridHeight - 1);
        j1 = Math.Clamp(j1, 0, gridHeight - 1);

        float sx = x - i0;
        float sy = yu - j0;

        float v00 = velocityFieldX[i0, j0];
        float v10 = velocityFieldX[i1, j0];
        float v01 = velocityFieldX[i0, j1];
        float v11 = velocityFieldX[i1, j1];

        float vx0 = v00 + sx * (v10 - v00);
        float vx1 = v01 + sx * (v11 - v01);
        return vx0 + sy * (vx1 - vx0);
    }
    private float SampleV(float x, float y)
    {
        float xv = x - 0.5f;

        int i0 = (int)MathF.Floor(xv);
        int j0 = (int)MathF.Floor(y);

        int i1 = i0 + 1;
        int j1 = j0 + 1;

        // clamp
        i0 = Math.Clamp(i0, 0, gridWidth - 1);
        i1 = Math.Clamp(i1, 0, gridWidth - 1);
        j0 = Math.Clamp(j0, 0, gridHeight);
        j1 = Math.Clamp(j1, 0, gridHeight);

        float sx = xv - i0;
        float sy = y - j0;

        float v00 = velocityFieldY[i0, j0];
        float v10 = velocityFieldY[i1, j0];
        float v01 = velocityFieldY[i0, j1];
        float v11 = velocityFieldY[i1, j1];

        float vy0 = v00 + sx * (v10 - v00);
        float vy1 = v01 + sx * (v11 - v01);
        return vy0 + sy * (vy1 - vy0);
    }



    // #######
    // DRAWING (alot of this will be wrong...cause I need to fix world/screen coords)
    // #######
    public void Draw()
    {
        //ComputeAndDrawStatistics();

        ComputeMinMaxDivergence();

        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Vector2 pos = new Vector2(x * cellSize, y * cellSize);
                DrawDivergence(x, y, pos);
                //DrawSquareCell(x, y, pos);
                DrawVelocityVectors(x, y, pos);
            }
        }
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                if (type[x, y] == 0) continue;
                Vector2 pos = new Vector2(x * cellSize, y * cellSize);
                DrawAdvectionVectors(x ,y, pos);
            }
        }
    }
    //private void ComputeAndDrawStatistics()
    //{
    //    // Compute
    //    maxSpeed = velocityFieldX.Max();

    //    // Draw
    //    int d = 10;
    //    void Stat(string label, float val)
    //    {
    //        Raylib.DrawText($"{label}: {val:E3}", 10, 10, 16, Raylib_cs.Color.White);
    //        d += 18;
    //    }
    //    Stat("dt", dt);
    //    Stat("max|u|", maxSpeed);
    //    //Stat("maxDivBefore", maxDivBefore);
    //    //Stat("maxDivAfter", maxDivAfter);
    //    //Stat("L2DivAfter", l2DivAfter);
    //    //Stat("mass", totalDyeMass);
    //}

    private void DrawAdvectionVectors(int x, int y, Vector2 pos)
    {
        float oldPosX = backtracedDataX[x, y].X * cellSize;
        float oldPosY = backtracedDataY[x, y].X * cellSize;

        float centerX = pos.X + 0.5f * cellSize;
        float centerY = pos.Y + 0.5f * cellSize;

        // this makes the visualization FALSE. Only use to see direction, not location
        float scale = 1f;

        oldPosX *= scale;
        oldPosY *= scale;

        // Mark old & new pos
        float size = (cellSize) / 32;
        Raylib.DrawCircle((int)centerX, (int)centerY, size, Raylib_cs.Color.DarkBlue);
        Raylib.DrawCircle((int)oldPosX, (int)oldPosY, size, Raylib_cs.Color.Red);

        // Draw a line to the backtraced (old) position
        Raylib.DrawLineEx(
            new Vector2(oldPosX, oldPosY),
            new Vector2(centerX, centerY),
            2f,
            Raylib_cs.Color.White
        );

        Raylib.DrawLineEx(
            new Vector2(oldPosX, oldPosY),
            new Vector2(backtracedDataX[x, y].Y + oldPosX, backtracedDataX[x, y].Z + oldPosY),
            2f,
            Raylib_cs.Color.Violet
        );
    }
    private void DrawVelocityVectors(int x, int y, Vector2 pos)
    {
        if (type[x, y] == 0) return;
        //DrawVelocityVectorX(x, y, pos);
        //DrawVelocityVectorY(x, y, pos);
        DrawVelocityVector(x, y, pos);
    }
    private void ComputeMinMaxDivergence()
    {
        minDiv = float.MaxValue;
        maxDiv = float.MinValue;

        for (int x = 0; x < gridWidth; x++)
            for (int y = 0; y < gridHeight; y++)
            {
                float d = divergence[x, y];
                if (d < minDiv) minDiv = d;
                if (d > maxDiv) maxDiv = d;
            }
    }
    private void DrawDivergence(int x, int y, Vector2 pos)
    {
        float div = divergence[x, y];

        // Normalize divergence to 0–1 range
        float normalized = (div - minDiv) / (maxDiv - minDiv + 1e-6f); // the addition is epsilon

        // Convert to color: blue = negative, red = positive, gray = near zero
        // Hue 0° = red, 240° = blue
        float hue = 240f * (1f - normalized);
        Raylib_cs.Color color = type[x, y] == 1 ? Raylib.ColorFromHSV(hue, 1f, 0.3f) : Raylib_cs.Color.Black;

        Raylib.DrawRectangle((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, color);
    }
    private void DrawSquareCell(int x, int y, Vector2 pos)
    {
        Raylib.DrawRectangleLines((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, Raylib_cs.Color.Black);
    }
    private void DrawVelocityVectorX(int x, int y, Vector2 pos)
    {
        int scale = 10;
        int yOffset = (int)(cellSize / 2);
        pos.Y += yOffset;
        Raylib.DrawLine((int)pos.X, (int)pos.Y,
                        (int)(pos.X + velocityFieldX[x, y] * scale),
                        (int)(pos.Y),
                        velocityFieldX[x, y] > 0 ? Raylib_cs.Color.White : Raylib_cs.Color.Black);
    }
    private void DrawVelocityVectorY(int x, int y, Vector2 pos)
    {
        int scale = 10;
        int xOffset = (int)(cellSize / 2);
        pos.X -= xOffset;
        Raylib.DrawLine((int)pos.X, (int)pos.Y,
                        (int)(pos.X),
                        (int)(pos.Y + velocityFieldY[x, y] * scale),
                        velocityFieldY[x, y] < 0 ? Raylib_cs.Color.White : Raylib_cs.Color.Black);
    }
    private void DrawVelocityVector(int x, int y, Vector2 pos)
    {
        int scale = 1;
        int offset = (int)(cellSize / 2);
        pos.X += offset;
        pos.Y += offset;
        Vector2 endPos = new Vector2(
            (pos.X + velocityFieldX[x, y] * scale),
            (pos.Y + velocityFieldY[x, y] * scale));

        Raylib.DrawLineEx(pos,
                        endPos,
                        1f,
                        Raylib_cs.Color.SkyBlue);
    }
}