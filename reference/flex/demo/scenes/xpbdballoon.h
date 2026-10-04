
// XPBD Inflatable Balloon (Macklin, Mueller, Chentanez, "XPBD: Position-Based
// Simulation of Compliant Constrained Dynamics", MIG 2016, Fig. 1 and the
// "Inflatable Balloon" sequence of the supplementary video)
//
// Four balloons rest on the floor. Each is a closed cloth surface (stretch and
// bending springs) with a global volume constraint holding its rest volume;
// the surface stiffness rises from left to right, so the softest balloon
// flattens under its own weight while the stiffest stays a sphere. The video
// compares 20 and 80 iterations: with XPBD the four shapes are the same at
// both counts.
//
// The surface coefficients map onto [stiffnessMin, stiffnessMax] N/m (the
// "Stiffness Min/Max" sliders, log10); the volume constraint's compliance is
// the global "Volume Compliance" slider (0 = incompressible).
//
// Measured at rest (4 substeps; height and centre height of each balloon,
// rest sphere 1.0 tall; enclosed volume stays 1.000 x rest in every case):
//   XPBD it20  height 0.43 0.52 0.70 0.87   centre 0.083 0.169 0.265 0.389
//   XPBD it80  height 0.43 0.52 0.70 0.87   centre 0.083 0.169 0.265 0.392
//   PBD  it20  keeps all four near-spherical (0.87 to 0.92 tall)
class XPBDInflatableBalloon : public Scene
{
public:

	XPBDInflatableBalloon(const char* name) : Scene(name) {}

	virtual ~XPBDInflatableBalloon()
	{
		for (size_t i = 0; i < mCloths.size(); ++i)
			delete mCloths[i];
	}

	void AddBalloon(const Mesh* mesh, float stiffness, int phase)
	{
		const int startVertex = g_buffers->positions.size();

		for (size_t i = 0; i < mesh->GetNumVertices(); ++i)
		{
			const Vec3 p = Vec3(mesh->m_positions[i]);

			g_buffers->positions.push_back(Vec4(p.x, p.y, p.z, 1.0f));
			g_buffers->velocities.push_back(0.0f);
			g_buffers->phases.push_back(phase);
		}

		const int triOffset = g_buffers->triangles.size();
		const int triCount = mesh->GetNumFaces();

		g_buffers->inflatableTriOffsets.push_back(triOffset / 3);
		g_buffers->inflatableTriCounts.push_back(triCount);
		g_buffers->inflatablePressures.push_back(1.0f);	// rest volume, no over-pressure

		for (size_t i = 0; i < mesh->m_indices.size(); i += 3)
		{
			const int a = mesh->m_indices[i + 0];
			const int b = mesh->m_indices[i + 1];
			const int c = mesh->m_indices[i + 2];

			const Vec3 n = -Normalize(Cross(mesh->m_positions[b] - mesh->m_positions[a], mesh->m_positions[c] - mesh->m_positions[a]));
			g_buffers->triangleNormals.push_back(n);

			g_buffers->triangles.push_back(a + startVertex);
			g_buffers->triangles.push_back(b + startVertex);
			g_buffers->triangles.push_back(c + startVertex);
		}

		// stretch and bending springs from the surface triangles, both with the
		// balloon's [0,1] coefficient
		ClothMesh* cloth = new ClothMesh(&g_buffers->positions[0], g_buffers->positions.size(), &g_buffers->triangles[triOffset], triCount * 3, stiffness, stiffness);

		for (size_t i = 0; i < cloth->mConstraintIndices.size(); ++i)
			g_buffers->springIndices.push_back(cloth->mConstraintIndices[i]);

		for (size_t i = 0; i < cloth->mConstraintCoefficients.size(); ++i)
			g_buffers->springStiffness.push_back(cloth->mConstraintCoefficients[i]);

		for (size_t i = 0; i < cloth->mConstraintRestLengths.size(); ++i)
			g_buffers->springLengths.push_back(cloth->mConstraintRestLengths[i]);

		mCloths.push_back(cloth);

		g_buffers->inflatableVolumes.push_back(cloth->mRestVolume);
		g_buffers->inflatableCoefficients.push_back(cloth->mConstraintScale);
	}

	void Initialize()
	{
		mCloths.resize(0);

		// softest to stiffest, left to right: on [10, 1e6] N/m these are
		// 316, 1.8e3, 1e4 and 1e5 N/m. With 642 unit-mass particles per balloon
		// the first flattens into a pancake, the last stays a sphere.
		const float stiffness[4] = { 0.35f, 0.5f, 0.65f, 0.85f };
		const float spacing = 2.0f;

		for (int i = 0; i < 4; ++i)
		{
			Mesh* mesh = ImportMesh(GetFilePathByPlatform("../../data/sphere.ply").c_str());
			mesh->Normalize();	// unit diameter, lower corner at the origin
			mesh->Transform(TranslationMatrix(Point3((i - 1.5f)*spacing - 0.5f, 0.6f, -0.5f)));

			AddBalloon(mesh, stiffness[i], NvFlexMakePhase(i, 0));

			delete mesh;
		}

		g_params.radius = 0.1f;
		g_params.dynamicFriction = 0.4f;
		g_params.dissipation = 0.0f;
		g_params.damping = 0.5f;		// light viscous drag so the balloons settle
		g_params.particleCollisionMargin = g_params.radius*0.05f;
		g_params.drag = 0.0f;
		g_params.collisionDistance = 0.01f;
		g_params.relaxationFactor = 1.0f;

		// unit-mass particles: 10 N/m flattens, 1e6 N/m is a hard shell
		g_params.solverMode = eNvFlexSolverXPBD;
		g_params.stiffnessMin = 10.0f;
		g_params.stiffnessMax = 1.0e6f;
		g_params.springDamping = 0.0f;
		g_params.volumeCompliance = 0.0f;
		g_params.numIterations = 20;

		// four substeps keep the stiffest balloon inside the regime where 20
		// Jacobi iterations converge
		g_numSubsteps = 4;

		g_windStrength = 0.0f;

		// draw options
		g_drawPoints = false;
		g_drawSprings = 0;
		g_drawCloth = false;
	}

	virtual void Sync()
	{
		NvFlexSetInflatables(g_solver, g_buffers->inflatableTriOffsets.buffer, g_buffers->inflatableTriCounts.buffer, g_buffers->inflatableVolumes.buffer, g_buffers->inflatablePressures.buffer, g_buffers->inflatableCoefficients.buffer, mCloths.size());
	}

	virtual void Draw(int pass)
	{
		if (!g_drawMesh)
			return;

		int indexStart = 0;

		for (size_t i = 0; i < mCloths.size(); ++i)
		{
			DrawCloth(&g_buffers->positions[0], &g_buffers->normals[0], NULL, &g_buffers->triangles[indexStart], mCloths[i]->mTris.size(), g_buffers->positions.size(), 3, g_params.radius*0.35f);

			indexStart += mCloths[i]->mTris.size() * 3;
		}
	}

	std::vector<ClothMesh*> mCloths;
};

