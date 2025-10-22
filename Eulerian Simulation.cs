using Raylib_cs;
using System.Drawing;
using System.Numerics;
// TODO: make seperate class for core and drawing, integrate into App after for Raylib functionality

class EulerianSimulation
{
    // Parameters
    private readonly int gridWidth;
    private readonly int gridHeight;
    /// <summary>
    /// cell size in pixels
    /// </summary>
    private readonly float cellSize;
    private Vector2[,] velocityField;
    public EulerianSimulation(int width, int height, float cellSize)
    {
        this.gridWidth = width;
        this.gridHeight = height;
        this.cellSize = cellSize;
        velocityField = new Vector2[gridWidth, gridHeight];
        InitializeField();
    }
    private void InitializeField() // staggard grid
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                velocityField[x, y] = new Vector2(0, 1); // Initialize velocities to zero
            }
        }
    }
    public void Update(float deltaTime)
    {

    }
    public void Draw()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Vector2 pos = new Vector2(x * cellSize, y * cellSize);
                DrawSquareCell(x, y, pos);
                DrawVelocityVector(x, y, pos);
            }
        }
    }

    private void DrawSquareCell(int x, int y, Vector2 pos)
    {
        Raylib.DrawRectangleLines((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, Raylib_cs.Color.Gray);
    }

    private void DrawVelocityVector(int x, int y, Vector2 pos)
    {
        Raylib.DrawLine((int)pos.X, (int)pos.Y,
                        (int)(pos.X + velocityField[x, y].X * 10),
                        (int)(pos.Y + velocityField[x, y].Y * 10),
                        Raylib_cs.Color.White);
    }
}