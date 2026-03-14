using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace CustomEulerianFluidSimulation
{
    public enum CellType
    {
        Solid = 0,
        Fluid = 1,
        Air = 2, // for later when we add free surface. For now we just have solid and fluid, and treat the outside as infinite fluid (fix later by generalizing the divergence computation and Poisson solver)
    }
    record SimDrawData
    {
        public float MinDivergence { get; init; }
        public float MaxDivergence { get; init; }
        public float MaxSpeed { get; init; }
        public float MinSpeed { get; init; }
        public float dt { get; init; }

    }
    record CellDrawData
    {
        public float Divergence { get; init; }
        public float Dye { get; init; }
        public Vector2 CellVelocity { get; init; }
        public CellType Type { get; init; }
        public Vector2 Position { get; init; } // indices of the cell, not world position
    }
}
