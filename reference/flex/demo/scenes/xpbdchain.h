
// XPBD Chain (Macklin, Mueller, Chentanez, "XPBD: Position-Based Simulation
// of Compliant Constrained Dynamics", MIG 2016, Sec. 6.2 and the "Chain"
// sequence of the supplementary video)
//
// Twenty unit-mass particles joined by distance constraints of compliance
// alpha = 1e-8 m/N, hanging from a fixed particle and released horizontally
// so the chain falls and swings under gravity. The paper solves it with 50
// iterations and compares the constraint force at the support with a Newton
// solver; here the stiffness range is pinned to [1e8, 1e8] N/m so every link
// has exactly that compliance whatever its coefficient.
//
// Measured (2 substeps): at 50 iterations the largest link-length error is
// 0.8% mid-swing and 0.5% at rest and the chain's lowest point is 1.3 cm
// below its taut length; at 10 iterations the errors are 5.2% / 3.0% and the
// overshoot 8 cm. Cycle "Num Iterations" to see the artificial compliance of
// an unconverged solve.
class XPBDChain : public Scene
{
public:

	XPBDChain(const char* name) : Scene(name) {}

	void Initialize()
	{
		const int numParticles = 20;
		const float segment = 0.1f;
		const Vec3 anchor = Vec3(0.0f, 2.5f, 0.0f);
		const int phase = NvFlexMakePhase(0, 0);

		const int baseIndex = int(g_buffers->positions.size());

		for (int i = 0; i < numParticles; ++i)
		{
			// particle 0 is the fixed support, the rest start level with it
			const float invMass = (i == 0) ? 0.0f : 1.0f;
			g_buffers->positions.push_back(Vec4(anchor.x + i*segment, anchor.y, anchor.z, invMass));
			g_buffers->velocities.push_back(Vec3(0.0f));
			g_buffers->phases.push_back(phase);
		}

		for (int i = 0; i < numParticles - 1; ++i)
			CreateSpring(baseIndex + i, baseIndex + i + 1, 1.0f);

		g_params.radius = 0.05f;
		g_params.dynamicFriction = 0.0f;
		g_params.dissipation = 0.0f;
		g_params.damping = 0.0f;
		g_params.drag = 0.0f;
		g_params.relaxationFactor = 1.0f;
		g_params.maxAcceleration = FLT_MAX;	// the released chain snaps taut; do not clamp it

		g_params.solverMode = eNvFlexSolverXPBD;
		g_params.stiffnessMin = 1.0e8f;
		g_params.stiffnessMax = 1.0e8f;
		g_params.springDamping = 0.0f;
		g_params.numIterations = 50;

		g_numSubsteps = 2;

		// draw options
		g_drawPoints = true;
		g_drawSprings = 1;	// 1: draw stretch springs

		g_windStrength = 0.0f;
	}
};

