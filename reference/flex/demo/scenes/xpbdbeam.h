
// XPBD Cantilever Beam (Macklin, Mueller, Chentanez, "XPBD: Position-Based
// Simulation of Compliant Constrained Dynamics", MIG 2016, Sec. 6.3 and the
// "Cantilever Beam" sequence of the supplementary video)
//
// The paper's beam is a St Venant-Kirchhoff triangular FEM (E = 1e5, Poisson's
// ratio 0.3) compared against a non-linear Newton solver. FleX has no FEM
// constraint, so this scene builds the beam as a 2D lattice of unit-mass
// particles joined by stretch, shear and bending springs, clamped along its
// left edge to a wall and bending under gravity with XPBD compliance. Cycle
// "Num Iterations" 20 -> 160: the tip deflection stays put in XPBD mode and
// keeps shrinking in PBD mode (the "Solver" slider). Measured tip deflection
// at rest (16 substeps, metres):
//   XPBD  it20 0.350  it40 0.349  it80 0.349  it160 0.349
//   PBD   it20 0.111                          it160 0.012
class XPBDCantileverBeam : public Scene
{
public:

	XPBDCantileverBeam(const char* name) : Scene(name) {}

	void Initialize()
	{
		const int dimx = 40;		// along the beam
		const int dimy = 8;			// through its thickness
		const float spacing = 0.05f;
		const float radius = spacing;

		// [0,1] coefficients map onto [stiffnessMin, stiffnessMax] = [1e3, 1e9] N/m
		// geometrically; 0.54 is ~1.7e6 N/m. With unit-mass particles and 16
		// substeps this keeps k*dt^2/m near 2, where FleX's Jacobi solver
		// converges within about 20 iterations (see xpbdcloth.h); the small
		// substep is what keeps the bend moderate at that convergence ratio.
		const float stretchStiffness = 0.54f;
		const float shearStiffness = 0.54f;
		const float bendStiffness = 0.54f;
		const int phase = NvFlexMakePhase(0, 0);

		const int baseIndex = int(g_buffers->positions.size());
		CreateSpringGrid(Vec3(0.0f, 0.0f, 0.0f), dimx, dimy, 1, spacing, phase, stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), 1.0f);

		// CreateSpringGrid lays the grid in the x-z plane (row index along z);
		// stand it up so the beam runs along +x with its thickness along y
		const float length = (dimx - 1)*spacing;
		const float thickness = (dimy - 1)*spacing;
		const float wallX = 0.5f;
		const float top = 2.0f;

		for (int i = baseIndex; i < int(g_buffers->positions.size()); ++i)
		{
			Vec4& p = g_buffers->positions[i];
			const int row = (i - baseIndex) / dimx;
			const int col = (i - baseIndex) % dimx;
			p = Vec4(wallX + col*spacing, top - row*spacing, 0.0f, p.w);
			g_buffers->velocities[i] = Vec3(0.0f);

			// clamp the whole left edge
			if (col == 0)
				p.w = 0.0f;
		}

		// the wall the beam is clamped to (decoration; the clamp is the pinned edge)
		AddBox(Vec3(0.2f, 0.6f, 0.4f), Vec3(wallX - 0.2f - radius, top - thickness*0.5f, 0.0f));

		g_params.radius = radius;
		g_params.dynamicFriction = 0.25f;
		g_params.dissipation = 0.0f;
		g_params.drag = 0.0f;
		g_params.damping = 0.5f;		// light viscous drag so the beam settles
		g_params.relaxationFactor = 1.0f;

		g_params.solverMode = eNvFlexSolverXPBD;
		g_params.stiffnessMin = 1.0e3f;
		g_params.stiffnessMax = 1.0e9f;
		g_params.springDamping = 0.0f;
		g_params.numIterations = 20;

		g_numSubsteps = 16;

		// draw options
		g_drawPoints = true;
		g_drawSprings = 1;	// 1: draw stretch springs

		g_windStrength = 0.0f;
	}
};

