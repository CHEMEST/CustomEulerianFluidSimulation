using Raylib_cs;
using System.Drawing;
using System.Numerics;

class EulerianSimulation
{
    // Parameters
    private readonly int gridWidth;
    private readonly int gridHeight;
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
    private Vector2[] bodyForces = new Vector2[1];
    // Simulation Constants
    Random random = new Random();
    private readonly Vector2 g = new Vector2(0, 9.81f); // gravity

    public EulerianSimulation(int width, int height, float cellSize)
    {
        this.gridWidth = width;
        this.gridHeight = height;
        this.cellSize = cellSize;
        velocityFieldX = new float[gridWidth + 1, gridHeight];
        velocityFieldY = new float[gridWidth, gridHeight + 1];
        InitializeVelocityField();

        bodyForces[0] = g;
    }
    private void InitializeVelocityField()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                velocityFieldX[x, y] = (float) random.NextDouble();
                velocityFieldY[x, y] = (float) random.NextDouble();
            }
        }
    }
    public void Update(float deltaTime)
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                //ApplyBodyForces(x, y, deltaTime);
                AdvectVelocity(x, y, deltaTime);
            }
        }
    }

    private void ApplyBodyForces(int x, int y, float dt)
    {
        foreach (var force in bodyForces)
        {
            velocityFieldX[x, y] += force.X * dt;
            velocityFieldY[x, y] += force.Y * dt;
        }
    }
    // is this even right ?
    private void AdvectVelocity(int x, int y, float dt)
    {
        float oldPosX = x - velocityFieldX[x, y] * dt;
        float oldPosY = y - velocityFieldY[x, y] * dt;
        Vector2 oldVel = BilinearSampleVelocity(oldPosX, oldPosY);
        velocityFieldX[x, y] = oldVel.X * dt;
        velocityFieldY[x, y] = oldVel.Y * dt;
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
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Vector2 pos = new Vector2(x * cellSize, y * cellSize);
                DrawSquareCell(x, y, pos);
                DrawVelocityVectors(x, y, pos);
            }
        }
    }

    private void DrawVelocityVectors(int x, int y, Vector2 pos)
    {
        DrawVelocityVectorX(x, y, pos);
        DrawVelocityVectorY(x, y, pos);
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
                        Raylib_cs.Color.White);
    }
    private void DrawVelocityVectorY(int x, int y, Vector2 pos)
    {
        int scale = 10;
        int xOffset = (int)(cellSize / 2);
        pos.X -= xOffset;
        Raylib.DrawLine((int)pos.X, (int)pos.Y,
                        (int)(pos.X),
                        (int)(pos.Y + velocityFieldY[x, y] * scale),
                        Raylib_cs.Color.White);
    }
}