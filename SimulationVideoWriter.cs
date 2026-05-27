using System.Diagnostics;
using System.IO;

// Almost entirely written by Anthropic Claude's Sonnet 4.5
namespace CustomEulerianFluidSimulation
{

    public class SimulationVideoWriter : IDisposable
    {
        private Process _ffmpeg;
        private Stream _stdin;
        private int gridWidth;
        private int gridHeight;
        private int cellSize;

        // width/height in cells, cellSize in pixels, framerate in fps
        public SimulationVideoWriter(string outputPath, int gridWidth, int gridHeight, int cellSize, int framerate = 30)
        {
            this.gridWidth = gridWidth;
            this.gridHeight = gridHeight;
            this.cellSize = cellSize;
            int pixelWidth = gridWidth * cellSize;
            int pixelHeight = gridHeight * cellSize;

            string args = string.Join(" ",
                "-f rawvideo",
                "-pixel_format rgb24",
                $"-video_size {pixelWidth}x{pixelHeight}",
                $"-framerate {framerate}",
                "-i pipe:0",
                "-c:v libx264",
                "-pix_fmt yuv420p", // required for broad MP4 compatibility
                "-y",               // overwrite output if exists
                $"\"{outputPath}\""
            );

            _ffmpeg = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "ffmpeg",
                    Arguments = args,
                    UseShellExecute = false,
                    RedirectStandardInput = true,
                    RedirectStandardError = true, // set true if you want FFmpeg log in C#
                    CreateNoWindow = true
                }
            };

            _ffmpeg.Start();
            _ffmpeg.ErrorDataReceived += (sender, e) => { if (e.Data != null) Console.WriteLine(e.Data); };
            _ffmpeg.BeginErrorReadLine();
            _stdin = _ffmpeg.StandardInput.BaseStream;
        }

        // gridColors: row-major array of (R, G, B) tuples, length == gridWidth * gridHeight
        // cellSize must match what was passed to the constructor
        public void WriteFrame((byte R, byte G, byte B)[] gridColors)
        {
            int pixelWidth = gridWidth * cellSize;
            int pixelHeight = gridHeight * cellSize;
            byte[] frameBuffer = new byte[pixelWidth * pixelHeight * 3];

            for (int cellY = 0; cellY < gridHeight; cellY++)
                for (int cellX = 0; cellX < gridWidth; cellX++)
                {
                    var (r, g, b) = gridColors[cellY * gridWidth + cellX];

                    for (int py = 0; py < cellSize; py++)
                        for (int px = 0; px < cellSize; px++)
                        {
                            int pixelIndex = ((cellY * cellSize + py) * pixelWidth + (cellX * cellSize + px)) * 3;
                            frameBuffer[pixelIndex] = r;
                            frameBuffer[pixelIndex + 1] = g;
                            frameBuffer[pixelIndex + 2] = b;
                        }
                }

            _stdin.Write(frameBuffer, 0, frameBuffer.Length);
        }

        public void Dispose()
        {
            _stdin.Close();
            _ffmpeg.WaitForExit();
            _ffmpeg.Dispose();
        }
    }
}
