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

        public App()
        {
            simulation = new EulerianSimulation(
                (WindowWidth / cellSize) - 1,
                (WindowHeight / cellSize) - 1);
            drawer = new Drawer(cellSize);
        }

        public void Draw()
        {
            drawer.DrawSim(simulation.GetCellDrawData(), simulation.GetSimDrawData());
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
