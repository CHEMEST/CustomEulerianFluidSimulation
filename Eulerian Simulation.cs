using Raylib_cs;
using System.Drawing;
using System.Numerics;

class EulerianSimulation
{
    // Parameters
    private readonly int gridWidth;
    private readonly int gridHeight;
    private readonly int pressureIters = 30;
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

    // For visualization
    float minDiv = float.MaxValue;
    float maxDiv = float.MinValue;

    // 0: position ; 1,2: velocity (x, y)
    Vector3[,] backtracedDataX;
    Vector3[,] backtracedDataY;

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
                RandomizeVelocities();
                divergence[x, y] = 0f;

                backtracedDataX[x, y] = Vector3.Zero;
                backtracedDataY[x, y] = Vector3.Zero;
                type[x, y] = (x == 0 || x == gridWidth - 1 || y == 0 || y == gridHeight - 1) ? 0 : 1; // borders are 0, inside is 1

            }
        }
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
    }
    public void Update(float deltaTime)
    {
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
                // Backtrace to find old position | x,y are indicies, not cooridinates
                if (type[x, y] == 0) continue;
                float centerX = (x + 0.5f) * cellSize;
                float centerY = (y + 0.5f) * cellSize;
                // the *cellSize feels right and makes sense + makes visualization make sense, but be wary of it
                float oldPosX = centerX - velocityFieldX[x, y] * dt * cellSize;
                float oldPosY = centerY - velocityFieldY[x, y] * dt * cellSize;

                Vector2 oldVel = BilinearSampleVelocity(oldPosX, oldPosY);
                backtracedDataX[x, y] = new Vector3(oldPosX, oldVel.X, oldVel.Y);
                backtracedDataY[x, y] = new Vector3(oldPosY, oldVel.X, oldVel.Y);

                velocityFieldX[x, y] = oldVel.X;
                velocityFieldY[x, y] = oldVel.Y;
            }
        }
    }


    // Bilinear sampling of velocity fields at (oldPosX, y)
    //input is world coordinates
    // Actively working on derivation
    private Vector2 BilinearSampleVelocity(float px, float py)
    {
        // find nearest velocity point to the input position (left-most center of the cell) (i, j)
        int ix = (int)Math.Floor(px / cellSize);
        int iy = (int)Math.Floor(py / cellSize);

        // positions of the left and right cell velocity vectors (i,j; i+1,j)
        float px0 = ix * cellSize;
        float px1 = (ix + 1) * cellSize;
        float py0 = iy * cellSize;
        float py1 = (iy + 1) * cellSize;

        // velocity values at those points
        float vx0 = velocityFieldX[(int)ix, (int)iy];
        float vx1 = velocityFieldX[(int)ix + 1, (int)iy];
        float vy0 = velocityFieldY[(int)ix, (int)iy];
        float vy1 = velocityFieldY[(int)ix, (int)iy + 1];

        // linear interpolation in x direction
        // V(C) = (C-A) * (V(B)-V(A))/(B-A) + V(A)
        float vxInterp = (px - px0) * (vx1 - vx0)/(px1 - px0) + vx0;
        float vyInterp = (py - py0) * (vy1 - vy0) / (py1 - py0) + vy0;

        return new Vector2(vxInterp, vyInterp);
    }

    public void Draw()
    {
        ComputeMinMaxDivergence();
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Vector2 pos = new Vector2(x * cellSize, y * cellSize);
                DrawDivergence(x, y, pos);
                DrawSquareCell(x, y, pos);
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
    private void DrawAdvectionVectors(int x, int y, Vector2 pos)
    {
        float oldPosX = backtracedDataX[x, y].X;
        float oldPosY = backtracedDataY[x, y].X;

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
        DrawVelocityVectorX(x, y, pos);
        DrawVelocityVectorY(x, y, pos);
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