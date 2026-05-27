using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace CustomEulerianFluidSimulation
{
    // The program controller. This class handles the window, screen, and chain of events, not specific computation

    // run these with ffmpeg installed on Cmd to generate a video with the recorded frames
    // ffmpeg -framerate 60 -i bin/Debug/net8.0/frame_%05d.png -c:v libx264 -pix_fmt yuv420p output.mp4
    // to delete the frames after making a video (don't forget to do this before pushing to git).
    // del frame_*.png 

    // for 128x128: (640x640 / 5x5) = 128x128
    class App
    {
        // Core Constants
        private const float scale = 1.75f;
        private const int WindowWidth = (int)(512 * scale);
        private const int WindowHeight = (int)(512 * scale);
        // Simulation Values
        private const int cellSize = 6;
        private readonly EulerianSimulation simulation;
        private readonly Drawer drawer;
        private static int steps = 0;
        private static bool simulating = false;
        private bool slowDown = false;

        private static bool recording = false;
        private static int recordedFrame = 0;


    public App()
        {
            int gridWidth = (WindowWidth / cellSize);
            int gridHeight = (WindowHeight / cellSize);
            simulation = new EulerianSimulation(gridWidth, gridHeight);
            drawer = new Drawer(cellSize, gridWidth, gridHeight, WindowWidth, WindowHeight);
        }

        public void Draw()
        {
            drawer.DrawSim(simulation.GetSimDrawData(), simulation.GetCellDrawData(), simulation.VelocityFieldX, simulation.VelocityFieldY,
                steps);
        }
        private void Update()
        {
            float dt = Raylib.GetFrameTime();
            steps += 1;
            simulation.Update(dt);
        }

        static void Main()
        {
            Raylib.InitWindow(WindowWidth, WindowHeight, "Incompressible 2D Eulerian Fluid");
            Raylib.SetTargetFPS(120);
            App app = new App();

            while (!Raylib.WindowShouldClose())
            {
                // --- Update ---
                if (simulating) app.Update();
                if (Raylib.IsKeyPressed(KeyboardKey.R))
                {
                    app.simulation.ResetSim();
                    steps = 0;
                    Console.WriteLine("Simulation Reset");
                }
                else if (Raylib.IsKeyPressed(KeyboardKey.Space))
                {
                    simulating = !simulating;
                    Console.WriteLine(simulating ? "Simulating" : "Paused");
                }
                else if (Raylib.IsKeyPressed(KeyboardKey.Enter))
                {
                    app.Update();
                }else if (Raylib.IsKeyPressed(KeyboardKey.U))
                {
                    app.simulation.InjectAndPerturbRed();
                    Console.WriteLine("Injected Red");
                }
                else if (Raylib.IsKeyPressed(KeyboardKey.I))
                {
                    app.simulation.InjectAndPerturbGreen();
                    Console.WriteLine("Injected Green");
                }else if (Raylib.IsKeyPressed(KeyboardKey.O))
                {
                    app.simulation.InjectAndPerturbBlue();
                    Console.WriteLine("Injected Blue");
                }else if (Raylib.IsKeyPressed(KeyboardKey.P))
                {
                    app.simulation.PerturbVelocity();
                    Console.WriteLine("Perturbed Velocity");
                }
                else if (Raylib.IsKeyPressed(KeyboardKey.V))
                {
                    app.drawer.ShowVelocityVectors = !app.drawer.ShowVelocityVectors;
                }else if (Raylib.IsKeyPressed(KeyboardKey.LeftShift))
                {
                    app.slowDown = !app.slowDown;
                    Raylib.SetTargetFPS(app.slowDown ? 10 : 60);
                }
                else if (Raylib.IsKeyPressed(KeyboardKey.Backslash))
                {
                    recording = !recording;

                    if (recording == true) app.drawer.StartRecording();
                    else app.drawer.FinishRecording();
                }


                // --- Draw ---
                Raylib.BeginDrawing();
                Raylib.ClearBackground(Color.DarkGray);

                app.Draw();

                Raylib.DrawFPS(10, 10);
                Raylib.EndDrawing();

                if (recording)
                {
                    //Image frame = Raylib.LoadImageFromScreen();
                    //Raylib.ExportImage(frame, $"frame_{recordedFrame:D5}.png");
                    //Raylib.UnloadImage(frame);
                    app.drawer.WriteFrameData(app.simulation.GetCellDrawData());
                    Console.WriteLine( recordedFrame++ );
                }

            }
            Raylib.CloseWindow();
        }
    }
}
