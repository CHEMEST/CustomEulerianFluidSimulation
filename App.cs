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
        private const int WindowWidth = 1400;
        private const int WindowHeight = 1000;
        // Simulation Values
        private const int cellSize = 64;
        private readonly EulerianSimulation simulation;
        private readonly Drawer drawer;
        private float time = 0f;
        private static bool active = false;
        private bool slowDown = false;

        public App()
        {
            int gridWidth = (WindowWidth / cellSize) - 1;
            int gridHeight = (WindowHeight / cellSize) - 1;
            simulation = new EulerianSimulation( gridWidth, gridHeight);
            drawer = new Drawer(cellSize, gridWidth, gridHeight);
        }

        public void Draw()
        {
            drawer.DrawSim(simulation.GetSimDrawData(), simulation.GetCellDrawData(), simulation.VelocityFieldX, simulation.VelocityFieldY);
        }
        private void Update()
        {
            float dt = Raylib.GetFrameTime();
            time += dt;
            simulation.Update(dt);
        }

        static void Main()
        {
            Raylib.InitWindow(WindowWidth, WindowHeight, "Incompressible 2D Eulerian Fluid");
            Raylib.SetTargetFPS(60);
            App app = new App();

            while (!Raylib.WindowShouldClose())
            { 
                // --- Update ---
                if (active) app.Update();
                if (Raylib.IsKeyPressed(KeyboardKey.R)){
                    app.simulation.RandomizeVelocities();
                } else if (Raylib.IsKeyPressed(KeyboardKey.Enter)){
                    app.Update();
                } else if (Raylib.IsKeyPressed(KeyboardKey.Space)){
                    active = !active;
                }
                else if (Raylib.IsKeyPressed(KeyboardKey.LeftShift))
                {
                    app.slowDown = !app.slowDown;
                    Raylib.SetTargetFPS(app.slowDown ? 10 : 60);
                }

                // --- Draw ---
                Raylib.BeginDrawing();
                Raylib.ClearBackground(Raylib_cs.Color.DarkGray);

                app.Draw();

                Raylib.DrawFPS(10, 10);
                Raylib.EndDrawing();
            }

            Raylib.CloseWindow();
        }
    }
}
