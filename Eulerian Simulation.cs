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
    /// Staggard grid setup (MAC)
    /// </summary>
    private float[,] velocityFieldX;
    /// <summary>
    /// Staggard grid setup (MAC)
    /// </summary>
    private float[,] velocityFieldY;
    private Vector2[] bodyForces = new Vector2[1];
    // Simulation Constants
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
                velocityFieldX[x, y] = 1f;
                velocityFieldY[x, y] = -1f;
            }
        }
    }
    public void Update(float deltaTime)
    {
        ApplyBodyForces();
    }

    private void ApplyBodyForces()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                foreach (var force in bodyForces)
                {
                    velocityFieldX[x, y] += force.X;
                    velocityFieldY[x, y] += force.Y;
                }
            }
        }
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
        Raylib.DrawRectangleLines((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, Raylib_cs.Color.Gray);
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