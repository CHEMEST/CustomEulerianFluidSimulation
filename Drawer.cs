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
        private float gridWidth;
        private float gridHeight;
        private Vector2 _offSet = new Vector2(0.75f);
        private const float eps = 1e-5f;
        public Drawer(int cellSize, float gridWidth, float gridHeight) 
        {
            this.cellSize = cellSize;
            this.gridWidth = gridWidth;
            this.gridHeight = gridHeight;
        }

        public void DrawSim(SimDrawData simDrawData, CellDrawData[,] cellDrawDatas, float[,] u, float[,] v, int steps)
        {            
            foreach (CellDrawData cellData in cellDrawDatas)
            {
                Vector2 pos = (cellData.Position + _offSet) * cellSize;
                DrawDivergence(pos, cellData.Divergence);
                DrawDye(pos, cellData.Dye);
                DrawCellVelocity(pos, cellData.CellVelocity);
                //DrawIndex(pos, cellData.Position);
                //if (cellData.Type == CellType.Solid)
                //{
                //    DrawSquareCell(pos);
                //}
            }
            //for (int i = 0; i <= gridWidth; i++)
            //{
            //    for (int j = 0; j < gridHeight; j++)
            //    {
            //        Vector2 pos = new Vector2((i + _offSet.X) * cellSize, (j + 0.5f + _offSet.Y) * cellSize);
            //        DrawVelocityU(pos, u[i, j]);
            //    }
            //}
            //for (int i = 0; i < gridWidth; i++)
            //{
            //    for (int j = 0; j <= gridHeight; j++)
            //    {
            //        Vector2 pos = new Vector2((i + 0.5f + _offSet.X) * cellSize, (j + _offSet.Y) * cellSize);
            //        DrawVelocityV(pos, v[i, j]);
            //    }
            //}
            DrawStatistics(simDrawData, steps);
        }

        private void DrawDye(Vector2 pos, float dye)
        {

            Raylib.DrawRectangle((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, new Raylib_cs.Color(dye, dye, dye, 0.5f));
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
                Raylib.DrawText($"{label}: {val:E3}", 10, d, 16, Raylib_cs.Color.White);
                d += 40;
            }
            Raylib.DrawRectangle(0, 0, 180, 40 + 40 * 6, new Raylib_cs.Color(0, 0, 0, 0.75f));
            Stat("steps", steps);
            Stat("dt", data.dt);
            Stat("Max|u|", data.MaxSpeed);
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
            // commented this out since borders should have div=0 anyway, and it's more interesting to see the color mapping even for solids. Also we can add a black border on top to make them visually distinct if needed.
            //if (type != CellType.Fluid)
            //{
            //    Raylib.DrawRectangle((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, Raylib_cs.Color.Black);
            //    return;
            //}

            // A fixed reference scale for what you consider "large" divergence.
            // Tune this once and then your colors are comparable across time.
            // Rule of thumb: start near typical post-projection residual you want to visualize.
            const float divScale = 0.1f;

            float mag = MathF.Abs(div);

            // Map magnitude to [0, 1) smoothly without needing maxDiv.
            // - linear near 0
            // - saturates for large magnitudes
            float a = MathF.Tanh(mag / divScale);

            // Direction -> hue (fixed endpoints).
            float hue = (div >= 0f) ? 342f : 215f; // red or blue

            // Magnitude -> saturation + value (brightness).
            // Near zero: dark gray-ish (low saturation, low value)
            // Large: vivid and bright
            float sat = 0.15f + 0.85f * a;
            float val = 0.10f + 0.90f * a;

            // Optional: dead-zone to make near-zero cells clearly neutral.
            // Use eps as your "close enough to zero" threshold.
            if (mag <= eps)
            {
                sat = 0.0f;
                val = 0.10f; // dark neutral
                hue = 0f;    // irrelevant when sat = 0
            }

            Raylib_cs.Color color = Raylib.ColorFromHSV(hue, sat, val);
            Raylib.DrawRectangle((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, color);
        }
        private void DrawSquareCell(Vector2 pos)
        {
            Raylib.DrawRectangleLines((int)pos.X, (int)pos.Y, (int)cellSize, (int)cellSize, Raylib_cs.Color.White);
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
        private void DrawCellVelocity(Vector2 pos, Vector2 velocity)
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
                            Raylib_cs.Color.SkyBlue);
        }
    }
}
