using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace CustomEulerianFluidSimulation
{
    // The program controller. This class handles the window, screen, and chain of events, not specific computation
    class App
    {
        // Core Constants
        private const int WindowWidth = 960;
        private const int WindowHeight = 540;
        // Usefuls
        public static readonly Vector2 center = new Vector2(WindowWidth / 2f, WindowHeight / 2f);
        // Simulation Values
        private readonly float cellSize = 64f;
        private readonly EulerianSimulation simulation;
        private float t = 0f;

        public App()
        {
            simulation = new EulerianSimulation(
                (int)(WindowWidth / cellSize),
                (int)(WindowHeight / cellSize),
                cellSize);
        }

        public void Draw()
        {
            simulation.Draw();
        }
        private void Update()
        {
            float dt = Raylib.GetFrameTime();
            t += dt;
            simulation.Update(dt);
        }

        static void Main()
        {
            Raylib.InitWindow(WindowWidth, WindowHeight, "Eulerian 2D Fluid");
            Raylib.SetTargetFPS(120);
            App app = new App();

            while (!Raylib.WindowShouldClose())
            { 
                // --- Update ---
                app.Update();
                if (Raylib.IsKeyPressed(KeyboardKey.R))
                {
                    app.simulation.RandomizeVelocities();
                }
                else if (Raylib.IsKeyPressed(KeyboardKey.Space))
                {
                    app.Update();
                }

                // --- Draw ---
                Raylib.BeginDrawing();
                Raylib.ClearBackground(Raylib_cs.Color.DarkGreen);

                app.Draw();

                Raylib.DrawFPS(10, 10);
                Raylib.EndDrawing();
            }

            Raylib.CloseWindow();
        }
    }
}
