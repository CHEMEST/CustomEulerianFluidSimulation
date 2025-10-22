using Raylib_cs;
using System.Drawing;
using System.Numerics;
// TODO: make seperate class for core and drawing, integrate into App after for Raylib functionality
class App
{
    // Core Constants
    private const int WindowWidth = 960;
    private const int WindowHeight = 540;
    // Usefuls
    public static readonly Vector2 center = new Vector2(WindowWidth / 2f, WindowHeight / 2f);
    // Simulation Values
    private float t = 0f;
    
    public void Draw()
    {
        // Example: Draw a moving circle based on time
        float radius = 50f;
        float x = center.X + (float)(100 * System.Math.Cos(t));
        float y = center.Y + (float)(100 * System.Math.Sin(t));
        Raylib.DrawCircle((int)x, (int)y, radius, Raylib_cs.Color.Red);
    }

    static void Main()
    {
        Raylib.InitWindow(WindowWidth, WindowHeight, "Eulerian 2D Fluid");
        Raylib.SetTargetFPS(120);

        while (!Raylib.WindowShouldClose())
        {
            // --- Update ---
            //t += Raylib.GetFrameTime();

            // --- Draw ---
            Raylib.BeginDrawing();
            Raylib.ClearBackground(Raylib_cs.Color.DarkBlue);

            //App.Draw();

            Raylib.DrawFPS(10, 10);
            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }
}
