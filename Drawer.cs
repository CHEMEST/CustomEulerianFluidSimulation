using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Reflection.Metadata;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace CustomEulerianFluidSimulation
{
    class Drawer
    {
        private float cellSize;
        private int gridWidth;
        private int gridHeight;
        private int screenWidth;
        private int screenHeight;
        private Vector2 _offSet = new Vector2(0.75f);

        public bool ShowVelocityVectors { get; set; } = false;
        private const float eps = 1e-5f;

        private SimulationVideoWriter? writer;
        private bool isRecording;

        private Texture2D densityTex;
        private Color[] pixels;
        public Drawer(int cellSize, int gridWidth, int gridHeight, int screenWidth, int screenHeight) 
        {
            this.cellSize = cellSize;
            this.gridWidth = gridWidth;
            this.gridHeight = gridHeight;
            this.screenWidth = screenWidth;
            this.screenHeight = screenHeight;

            Image img = Raylib.GenImageColor(gridWidth, gridHeight, Color.Black);
            densityTex = Raylib.LoadTextureFromImage(img);
            Raylib.UnloadImage(img);
            Raylib.SetTextureFilter(densityTex, TextureFilter.Point); // sharp pixel blocks

            pixels = new Color[gridWidth * gridHeight];
        }

        public void WriteFrameData(CellDrawData[,] cellDrawDatas)
        {
            if (isRecording && writer != null)
            {
                writer?.WriteFrame(
                gridColors: cellDrawDatas.Cast<CellDrawData>().Select(cd => (
                    R: (byte)(cd.Ink.X * 255),
                    G: (byte)(cd.Ink.Y * 255),
                    B: (byte)(cd.Ink.Z * 255)
                )).ToArray()
            );
            }
        }

        public void DrawSim(SimDrawData simDrawData, CellDrawData[,] cellDrawDatas, float[] u, float[] v, int steps)
        {
            unsafe
            {
                for (int i = 0; i < gridWidth * gridHeight; i++)
                {
                    byte r = (byte)(255f - MathF.Max(0f, MathF.Min(simDrawData.InkR[i] * 255f, 255f)));
                    byte g = (byte)(255f - MathF.Max(0f, MathF.Min(simDrawData.InkG[i] * 255f, 255f)));
                    byte b = (byte)(255f - MathF.Max(0f, MathF.Min(simDrawData.InkB[i] * 255f, 255f)));

                    pixels[i] = new Color(r, g, b, (byte)255);
                }

                fixed (Color* ptr = pixels)
                    Raylib.UpdateTexture(densityTex, ptr);
            }

            Raylib.DrawTexturePro(
                densityTex,
                new Rectangle(0, 0, gridWidth, gridHeight),
                new Rectangle(0, 0, screenWidth, screenHeight),
                Vector2.Zero,
                0f,
                Color.White
            );

            if (ShowVelocityVectors)
                foreach (CellDrawData cellData in cellDrawDatas)
            {
                Vector2 pos = (cellData.Position + _offSet) * cellSize;
                //DrawDye(pos, Vector3.One - cellData.Ink);
                //DrawDivergence(pos, cellData.Divergence);
                 DrawCellVelocity(pos, cellData.CellVelocity, cellData.Vorticity);
                //DrawIndex(pos, cellData.Position);
                //if (cellData.Type == CellType.Solid)
                //{
                //DrawSquareCell(pos);
                //}
            }

            // --- for velocities on faces ---
            //for (int i = 0; i <= gridWidth; i++)
            //{
            //    for (int j = 0; j < gridHeight; j++)
            //    {
            //        Vector2 pos = new Vector2((i + _offSet.X) * cellSize, (j + 0.5f + _offSet.Y) * cellSize);
            //        DrawVelocityU(pos, u[i * gridWidth + j]);
            //    }
            //}
            //for (int i = 0; i < gridWidth; i++)
            //{
            //    for (int j = 0; j <= gridHeight; j++)
            //    {
            //        Vector2 pos = new Vector2((i + 0.5f + _offSet.X) * cellSize, (j + _offSet.Y) * cellSize);
            //        DrawVelocityV(pos, v[i * gridWidth + j]);
            //    }
            //}
            DrawStatistics(simDrawData, steps);
        }

        private void DrawDye(Vector2 pos, Vector3 dye)
        {
            Raylib.DrawRectangle((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, new Raylib_cs.Color(dye.X, dye.Y, dye.Z, 1f));
        }

        private void DrawIndex(Vector2 pos, Vector2 index)
        {
            Raylib.DrawText($"{index.X},{index.Y}", (int)pos.X + 5, (int)pos.Y + 5, 10, Raylib_cs.Color.White);
        }

        private void DrawStatistics(SimDrawData data, int steps)
        {
            int d = 40;
            void Stat(string label, float val)
            {
                Raylib.DrawText($"{label}: {val:E1}", 10, d, 16, Raylib_cs.Color.White);
                d += 25;
            }
            Raylib.DrawRectangle(0, 0, 160, 40 + 25 * 6, new Raylib_cs.Color(0, 0, 0, 0.5f));
            Stat("steps", steps);
            Stat("dt", data.Dt);
            Stat("Max|u,v|", data.MaxSpeed);
            Stat("MinDiv", data.MinDivergence);
            Stat("MaxDiv", data.MaxDivergence);
            Stat("Dye", data.TotalDye);
        }
        //private void DrawAdvectionVectors(int x, int y, Vector2 pos)
        //{
        //    float oldPosX = backtracedDataX[x, y].X * cellSize;
        //    float oldPosY = backtracedDataY[x, y].X * cellSize;

        //    float centerX = pos.X + 0.5f * cellSize;
        //    float centerY = pos.Y + 0.5f * cellSize;

        //    // this makes the visualization FALSE. Only use to see direction, not location
        //    float scale = 1f;

        //    oldPosX *= scale;
        //    oldPosY *= scale;

        //    // Mark old & new pos
        //    float size = (cellSize) / 32;
        //    Raylib.DrawCircle((int)centerX, (int)centerY, size, Raylib_cs.Color.DarkBlue);
        //    Raylib.DrawCircle((int)oldPosX, (int)oldPosY, size, Raylib_cs.Color.Red);

        //    // Draw a line to the backtraced (old) position
        //    Raylib.DrawLineEx(
        //        new Vector2(oldPosX, oldPosY),
        //        new Vector2(centerX, centerY),
        //        2f,
        //        Raylib_cs.Color.White
        //    );

        //    Raylib.DrawLineEx(
        //        new Vector2(oldPosX, oldPosY),
        //        new Vector2(backtracedDataX[x, y].Y + oldPosX, backtracedDataX[x, y].Z + oldPosY),
        //        2f,
        //        Raylib_cs.Color.Violet
        //    );
        //}

        // ChatGPT helped me write this function to visualize the divergence field with a nice color mapping.
        private void DrawDivergence(Vector2 pos, float div)
        {
            const float divScale = 0.1f;
            float mag = MathF.Abs(div);
            float a = MathF.Tanh(mag / divScale);
            float hue = (div >= 0f) ? 342f : 215f;
            float sat = 0.15f + 0.85f * a;
            float val = 0.10f + 0.90f * a;

            if (mag <= eps)
            {
                sat = 0.0f;
                val = 0.10f;
                hue = 0f;
            }

            Raylib_cs.Color color = Raylib.ColorFromHSV(hue, sat, val);
            color.A = (byte)(0.2f * 255);
            Raylib.DrawRectangle((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, color);
        }
        private Raylib_cs.Color ComputeVorticityColor(float omega)
        {
            const float scale = 0.1f;
            float mag = MathF.Abs(omega);
            float a = MathF.Tanh(mag / scale);
            float hue = (omega >= 0f) ? 300f : 190f;
            //float hue = 55f;
            float sat = 1f - (0.15f + 0.85f * a);
            float val = 1f - (0.10f + 0.90f * a);

            if (mag <= eps)
            {
                sat = 0.0f;
                val = 0.10f;
                hue = 0f;
            }

            Raylib_cs.Color color = Raylib.ColorFromHSV(hue, sat, val);
            color.A = (byte)(0.5f * 255);
            return color;
        }
        private void DrawSquareCell(Vector2 pos)
        {
            Raylib.DrawRectangleLines((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, new Raylib_cs.Color(1, 1, 1, 0.1f));
        }
        private void DrawVelocityU(Vector2 pos, float velocity)
        {
            int scale = 10;
            Raylib.DrawLine((int)pos.X, (int)pos.Y,
                            (int)(pos.X + velocity * scale),
                            (int)(pos.Y),
                            velocity > 0 ? new Raylib_cs.Color(0, 1, 0, 0.5f) : new Raylib_cs.Color(1, 0, 0, 0.5f));
        }
        private void DrawVelocityV(Vector2 pos, float velocity)
        {
            int scale = 10;
            Raylib.DrawLine((int)pos.X, (int)pos.Y,
                            (int)(pos.X),
                            (int)(pos.Y + velocity * scale),
                            velocity < 0 ? Raylib_cs.Color.White : Raylib_cs.Color.Black);
        }
        private void DrawCellVelocity(Vector2 pos, Vector2 velocity, float vorticity)
        {

            int scale = 10;
            int offset = (int)(cellSize / 2);
            pos += new Vector2(offset, offset);
            Vector2 endPos = new Vector2(
                (pos.X + velocity.X * scale),
                (pos.Y + velocity.Y  * scale));

            Raylib.DrawLineEx(pos,
                            endPos,
                            2f,
                            ComputeVorticityColor(vorticity));
        }

        public void StartRecording()
        {
            if (isRecording) return;
            string projectFolder = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\"));
            string outputPath = Path.Combine(projectFolder, "simulation.mp4");
            writer = new SimulationVideoWriter(outputPath, gridWidth, gridHeight, (int)cellSize);
            isRecording = true;
            Console.WriteLine("Started recording simulation video.");
        }

        public void FinishRecording()
        {
            if (!isRecording) return;
            writer?.Dispose();
            writer = null;
            isRecording = false;
            Console.WriteLine("Finished recording simulation video.");
        }
        public void FinishDrawing()
        {
            // --- Cleanup ---
            Raylib.UnloadTexture(densityTex);
        }
    }
}
