using Raylib_cs;
using System.Drawing;
using System.Numerics;

class EulerianSimulation
{
    // Parameters
    private readonly int gridWidth;
    private readonly int gridHeight;
    private readonly int pressureIters = 40;
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
    private readonly Vector2 g = new Vector2(0, -9.81f); // gravity

    // For visualization
    float minDiv = float.MaxValue;
    float maxDiv = float.MinValue;

    float[,] oldPosX;
    float[,] oldPosY;

    public EulerianSimulation(int width, int height, float cellSize)
    {
        this.gridWidth = width;
        this.gridHeight = height;
        this.cellSize = cellSize;
        velocityFieldX = new float[gridWidth + 1, gridHeight];
        velocityFieldY = new float[gridWidth, gridHeight + 1];
        divergence = new float[gridWidth, gridHeight];
        type = new int[gridWidth, gridHeight];

        oldPosX = new float[gridWidth + 1, gridHeight];
        oldPosY = new float[gridWidth, gridHeight + 1];

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

                oldPosX[x, y] = 0f;
                oldPosY[x, y] = 0f;
                // Something is WRONG here
                type[x, y] = (x == 0 || x == gridWidth || y == 0 || y == gridHeight) ? 0 : 1; // borders are 0, inside is 1

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
        //AdvectVelocity(deltaTime);

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

    private void SolvePoissonPressure(float dt) // WROOOOOOOOOOOOOOOOONG
    {
        for (int n = 0; n < pressureIters; n++)
            for (int x = 1; x < gridWidth - 1; x++)
            {
                for (int y = 1; y < gridHeight - 1; y++)
                {
                    float div = divergence[x, y];
                    float s = type[x + 1, y] + type[x - 1, y] + type[x, y + 1] + type[x, y - 1];

                    velocityFieldX[x, y] += dt * (div * type[x - 1, y] / s);
                    velocityFieldX[x + 1, y] += dt * (-div * type[x + 1, y] / s);
                    velocityFieldY[x, y] += dt * (div * type[x, y - 1] / s);
                    velocityFieldY[x, y + 1] += dt * (-div * type[x, y + 1] / s);
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
                float centerX = (x + 0.5f) * cellSize;
                float centerY = (y + 0.5f) * cellSize;
                float oldPosX1 = centerX - velocityFieldX[x, y] * dt;
                float oldPosY1 = centerY - velocityFieldY[x, y] * dt;
                oldPosX[x, y] = oldPosX1;
                oldPosY[x, y] = oldPosY1;

                //Vector2 oldVel = BilinearSampleVelocity(oldPosX, oldPosY);
                //velocityFieldX[x, y] = oldVel.X * dt;
                //velocityFieldY[x, y] = oldVel.Y * dt;
            }
        }
    }


    //scrap this slop and make it from scratch (TODO REMAKE THIS)
    // Bilinear sampling of velocity fields at (oldPosX, y)
    private Vector2 BilinearSampleVelocity(float oldPosX, float oldPosY)
    {
        // Clamp coordinates to valid range
        float x = Math.Clamp(oldPosX, 0, gridWidth - 1);
        float y = Math.Clamp(oldPosY, 0, gridHeight - 1);

        // Calc the change in cords
        int x0 = (int)Math.Floor(x);
        int x1 = Math.Min(x0 + 1, gridWidth - 1);
        int y0 = (int)Math.Floor(y);
        int y1 = Math.Min(y0 + 1, gridHeight - 1);

        float tx = x - x0;
        float ty = y - y0;

        // Sample velocityFieldX (staggered: [gridWidth+1, gridHeight])
        float vx00 = velocityFieldX[x0, y0];
        float vx10 = velocityFieldX[x1, y0];
        float vx01 = velocityFieldX[x0, y1];
        float vx11 = velocityFieldX[x1, y1];
        float vx0 = vx00 * (1 - tx) + vx10 * tx;
        float vx1 = vx01 * (1 - tx) + vx11 * tx;
        float vx = vx0 * (1 - ty) + vx1 * ty;

        // Sample velocityFieldY (staggered: [gridWidth, gridHeight+1])
        float vy00 = velocityFieldY[x0, y0];
        float vy10 = velocityFieldY[x1, y0];
        float vy01 = velocityFieldY[x0, y1];
        float vy11 = velocityFieldY[x1, y1];
        float vy0 = vy00 * (1 - tx) + vy10 * tx;
        float vy1 = vy01 * (1 - tx) + vy11 * tx;
        float vy = vy0 * (1 - ty) + vy1 * ty;

        return new Vector2(vx, vy);
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
                Vector2 pos = new Vector2(x * cellSize, y * cellSize);
                //DrawAdvectionVectors(oldPosX[x, y], oldPosY[x, y], pos);
            }
        }


    }
    private void DrawAdvectionVectors(float oldPosX, float oldPosY, Vector2 pos)
    {
        float centerX = pos.X + 0.5f * cellSize;
        float centerY = pos.Y + 0.5f * cellSize;

        // this makes the visualization FALSE. Only use to see direction, not location. Also this is shit code lmfao
        float scale = 2f;
        if (scale != 1f)
        {
            oldPosX -= 0.5f * cellSize;
            oldPosY -= 0.5f * cellSize;
            oldPosX += 0.5f * cellSize * scale;
            oldPosY += 0.5f * cellSize * scale;
        }

        // Draw a line to the backtraced (old) position
        Raylib.DrawLineEx(
            new Vector2(oldPosX, oldPosY),
            new Vector2(centerX, centerY),
            2f,
            Raylib_cs.Color.White
        );

        // Mark old & new pos
        Raylib.DrawCircle((int)centerX, (int)centerY, 3f, Raylib_cs.Color.DarkBlue);
        Raylib.DrawCircle((int)oldPosX, (int)oldPosY, 3f, Raylib_cs.Color.Red);
    }
    private void DrawVelocityVectors(int x, int y, Vector2 pos)
    {
        DrawVelocityVectorX(x, y, pos);
        DrawVelocityVectorY(x, y, pos);
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
        Raylib_cs.Color color = type[x, y] == 1 ? Raylib.ColorFromHSV(hue, 1f, 1f) : Raylib_cs.Color.Black;

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
}