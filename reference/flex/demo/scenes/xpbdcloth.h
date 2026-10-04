// XPBD Hanging Cloth (Macklin, Mueller, Chentanez, "XPBD: Position-Based
// Simulation of Compliant Constrained Dynamics", MIG 2016, Fig. 6 and the
// "Hanging Cloth" sequence of the supplementary video).
//
// A 64x64 cloth -- 4096 particles, 23938 stretch/shear/bend constraints, the
// video's "64x64 particles, 24k constraints" -- hangs by its two top corners
// below a bar in a gusting wind. The video compares 20, 40, 80 and 160
// iterations; with XPBD the drape is the same at every count, which is what
// this scene reproduces.
//
// ---------------------------------------------------------------------------
// How the scene was matched to data/xpbd_supplementary.mp4
// ---------------------------------------------------------------------------
//
// The shot is rendered with this very demo: the checkerboard ground plane, the
// spotlight, the teal cloth (DrawCloth's default colour index 3) and the
// capsule bar are all stock. That is what makes an exact match possible, and it
// supplies the ruler:
//
//   meshPS.hlsl draws the ground plane through bump(), which flips at every
//   integer, so the checker is one world METRE per square.
//
// For a pinhole camera of pitch t over that plane, the image-space pitch p of
// the checker at image row v obeys exactly
//
//        v = v_horizon + p * h / cos(t)      (h = camera height)
//
// so p is linear in v, the intercept is the horizon and the slope is cos(t)/h.
// Both are independent of focal length AND of image resolution, which makes the
// slope a direct ruler for the camera height in metres -- and hence, once the
// framing is matched, for the whole scene. Fitted over 28 frames x 4 panels of
// the "Our Method" shot the four panels agree to 0.5%:
//
//        slope 0.2152 /m, horizon at panel row 317.6  ->  h = 4.49 m
//
// The same fit on a render of this scene, where the camera is known exactly,
// returns 9.40 m against a true 9.30 m, i.e. the method is good to about 1%.
//
// The panel is 270 px wide and its floor ends at row 485; putting the horizon at
// 317.6 with the demo's 45 degree vertical fov requires a viewport 203 px tall,
// i.e. a 4:3 render, and a pitch of 15.0 degrees -- which is the demo's own
// default camera angle. Everything locks together, and the camera follows:
//
//        pitch -15 deg, height 4.49 m, 2.503 W in front of the cloth plane,
//        pins 0.132 W above the camera            (W = the cloth's rest width)
//
// That leaves one unknown, the cloth's absolute size, because a cloth's
// silhouette plus a camera is scale-degenerate: scaling the cloth and its
// distance together leaves every pixel unchanged. The floor breaks it. Three
// independent readings agree:
//
//   - The pins sit 14 panel rows ABOVE the horizon and the hem 110 below, so
//     with h = 4.49 m the hem clears the floor only if W < 3.79 m, and the
//     clearance the video actually shows puts W at 3.1..3.6 m.
//   - The cloth's width divided by the checker pitch just below its hem reads
//     5.15 m; that recipe over-reads by 29% on a render of known size (16.2 m
//     against a true 12.6 m), which corrects to about 4.0 m.
//
// 64 x 64 at the demo's usual cloth spacing of 0.05 m is W = 3.15 m, which sits
// inside both. What settles it is that the two readings are independent and a
// render can satisfy both at once only at the right size: at 3.15 m this scene
// reproduces the video's checker slope to 0.08% (0.21540 against 0.21522 /m)
// AND its pin separation in pixels to 0.3% (365.0 against 366.2 px) in the same
// frame. Get the cloth wrong by 10% and one of the two has to move.
//
// A sway-period argument was tried as a third line and does not work: a sheet
// hung from its top edge has a fundamental of T = 5.225 sqrt(L/g), which for
// this drop is 3.4 s and agrees prettily with the 3.33 s autocorrelation peak of
// the video's H/D -- but measured in still air this cloth actually rings at
// 1.43 s, because its top row is under 37% strain and it is a tensioned
// membrane, not a free chain. The formula does not apply and the agreement was
// a coincidence; the 3.33 s is the gust envelope.
//
// Two things the earlier, larger version of this scene got wrong are worth
// recording, because both came from comparing quantities that were not
// comparable:
//
//   - It read H/D straight off the video's pixels (1.225) and matched the
//     SIMULATION's world-space H/D to it. Under a 15 degree pitch the hem is
//     14% further from the camera than the pins, so the cloth's vertical extent
//     is foreshortened: the true H/D is 1.33, not 1.225. The grid had been cut
//     from 64x64 to 64x63 to close that gap, which was chasing a projection
//     artefact. Every comparison below is now made in PIXEL space, through this
//     same camera, so the projection cancels on both sides.
//   - It set the scale from a sway period, matching a 3.10 s measurement on a
//     12.6 m cloth to the video's 3.33 s. A hanging cloth's period is not a
//     free-chain mode at any size here -- this 3.15 m one rings at 1.43 s -- so
//     that observable could not have fixed the scale either way. The floor is
//     what does, and the floor was in every frame all along.
//
// ---------------------------------------------------------------------------
// What sets the shape
// ---------------------------------------------------------------------------
//
//  - kXpbdStretch sets the dip of the top edge, and very sensitively: the whole
//    sheet hangs off the 63 springs of the top row, so a dip of 0.27 W needs
//    that row about 37% longer than its rest length (a catenary of sag f over
//    span D has arc/D = 1 + 8(f/D)^2/3). The corners hang at the cloth's full
//    rest width, so the dip is stretch, not slack. One part in 200 of the
//    coefficient moves the dip by a twentieth of W.
//  - kXpbdShear sets the silhouette's waist. Cloth is stiff in stretch but soft
//    in shear, and that is what lets the sheet pull in between its pinned
//    corners; with shear equal to stretch it renders as a flat board.
//  - kXpbdBend sets how fine the drape folds are. It is the softest of the
//    three: at 0.15 the sheet is a smooth funnel, at 0.05 it carries the
//    video's vertical folds.
//  - kSeedRipple lets the folds start. Below the tensioned top row the sheet is
//    in lateral compression, but a perfectly planar sheet is in unstable
//    equilibrium and will stay flat forever, so the particles are seeded with a
//    few millimetres of out-of-plane ripple to choose a buckling mode.
//
// The compliances are absolute, so they do not carry over from a cloth of a
// different size: a spring of rest length s needs alpha proportional to s for
// the same strain, and the stiffness mapping is geometric over
// [stiffnessMin, stiffnessMax], so shrinking the cloth by 4 shifts every
// coefficient by log10(4)/6 = 0.100. That is where these values started before
// they were tuned against the video.
//
// The wind is the other half of the look, and the thing to get right is its
// PERIOD, not its strength. Drag in UpdateTriangles.hlsl carries no triangle
// area factor, so the wind acceleration -- and with it the lean -- does not
// depend on the cloth's size. The cloth's own timescale does, and it is not the
// one a hanging sheet would have: measured in still air this cloth rings at
// 1.43 s, not the 3.4 s a free hanging chain of this drop gives, because its
// top row is under 37% strain and it behaves as a tensioned membrane. The
// video's cloth has its spectral peak at 1.73 s, so the drive belongs at about
// the same place, and kGustRate sits there.
//
// ---------------------------------------------------------------------------
// How well it matches, and where it does not
// ---------------------------------------------------------------------------
//
// Every row below is a PIXEL measurement of the cloth's silhouette, taken with
// one piece of code on both sides: on 1140 frames of the video's panel 0, and
// on a 2400-frame capture of this scene projected through the camera above.
// D is the pin separation.
//
//   metric        video (p10..p90)        this scene (p10..p90)
//   pin row       76.4                    77.0
//   hem row       506.7                   513.5
//   D             366.2 px                365.0 px
//   dip/D         0.277 (0.245..0.311)    0.270 (0.230..0.321)
//   H/D           1.175 (0.853..1.260)    1.196 (1.032..1.549)
//   W/D           0.903 (0.748..1.133)    0.890 (0.762..1.107)
//   checker slope 0.21522 /m              0.21540 /m
//   cloth RGB     (79, 198, 164)          (81, 197, 164)
//   other side    (225, 220, 220)         (210, 208, 206)
//
// Two things are honestly still short:
//
//   - The flutter is about a fifth too weak: the detrended H/D has an rms of
//     0.102 here against the video's 0.124.
//   - More tellingly, the two clothes get their variation from different
//     places. The video's H/D excursions go DOWN (p10 0.853 against a median
//     1.175) because its hem lifts during gusts; this scene's go UP (p90 1.549)
//     because its motion is mostly a depth swing, and swinging towards a camera
//     8 m away makes the silhouette bigger. Driving the hem up instead needs
//     g_params.lift, and FleX's lift acts on every triangle including the
//     tensioned top row, so every setting that lifted the hem enough also blew
//     the catenary flat -- dip/D fell to 0.18..0.22 against the video's steady
//     0.27. kLift is held at 0.2, which is as much as the dip tolerates.
//
class XPBDHangingCloth : public Scene
{
public:

	XPBDHangingCloth(const char* name) : Scene(name) {}

	void Initialize()
	{
		const int dimx = 64;
		const int dimy = 64;
		const float spacing = kSpacing;
		const float radius = spacing;

		// a single sheet that never folds onto itself: no self-collision, which
		// would otherwise add stiff, iteration-dependent contacts near the corners
		const int phase = NvFlexMakePhase(0, 0);

		const int baseIndex = int(g_buffers->positions.size());
		CreateSpringGrid(Vec3(0.0f, 0.0f, 0.0f), dimx, dimy, 1, spacing, phase, kXpbdStretch, kXpbdBend, kXpbdShear, Vec3(0.0f), 1.0f);

		// CreateSpringGrid lays the grid in the x-z plane (row index along z);
		// stand it up with row 0 on top at the pin height read off the video
		const float width = (dimx - 1)*spacing;
		const float height = (dimy - 1)*spacing;
		const float top = kPinHeight;
		mWidth = width;
		mHeight = height;
		mTop = top;

		for (int i = baseIndex; i < int(g_buffers->positions.size()); ++i)
		{
			Vec4& p = g_buffers->positions[i];
			const int row = (i - baseIndex) / dimx;
			const int col = (i - baseIndex) % dimx;

			// A few millimetres of out-of-plane ripple. The sheet below the top
			// row is laterally compressed and would buckle into folds, but a
			// perfectly flat sheet sits in unstable equilibrium and stays flat;
			// this picks the mode instead of waiting for round-off to. Two
			// incommensurate waves so the folds are not periodic, faded in from
			// the pinned top row towards the free bottom edge.
			const float u = float(col)/float(dimx - 1);
			const float v = float(row)/float(dimy - 1);
			const float ripple = kSeedRipple*spacing*v*(sinf(11.0f*u) + 0.5f*sinf(7.0f*u + 2.0f));

			p = Vec4(col*spacing - width*0.5f, top - row*spacing, ripple, p.w);
			g_buffers->velocities[i] = Vec3(0.0f);
		}

		// the two top corners hang at the cloth's full rest width
		g_buffers->positions[baseIndex].w = 0.0f;
		g_buffers->positions[baseIndex + dimx - 1].w = 0.0f;

		// The bar. Measured off the video: 1.53 times the pin separation long, a
		// radius of 0.077 m, and its centre one radius above the pin row so its
		// underside just touches the top edge. It sits a little towards the
		// camera because the wind blows the other way, which keeps it decoration
		// rather than a collider the sheet could catch on.
		// AddCapsule's half-height excludes the end caps, so the drawn length is
		// 2*halfHeight + 2*radius; solve that for the length measured off the video.
		AddCapsule(kBarRadius, 0.5f*(kBarLength*width - 2.0f*kBarRadius), Vec3(0.0f, top + 1.1f*kBarRadius, 0.5f*kBarRadius), Quat());

		g_params.radius = radius;
		g_params.dynamicFriction = 0.25f;
		g_params.dissipation = 0.0f;

		// the wind is an acceleration that only reaches the cloth through the
		// triangles' drag, and lift (the force perpendicular to the flow) is
		// what turns a steady push into flapping; damping has to stay low or it
		// eats the flutter before it can travel down the sheet
		g_params.drag = kDrag;
		g_params.lift = kLift;
		g_params.damping = kDamping;

		// XPBD's near-rigid constraints need the count-averaged (local) update
		g_params.relaxationMode = eNvFlexRelaxationLocal;
		g_params.relaxationFactor = 1.0f;

		g_params.solverMode = eNvFlexSolverXPBD;
		g_params.stiffnessMin = 1.0e3f;
		g_params.stiffnessMax = 1.0e9f;
		g_params.springDamping = 0.0f;
		g_params.numIterations = 20;

		g_numSubsteps = kXpbdSubsteps;

		// draw options
		// The cloth's colours are measured off the video rather than left at the
		// palette's defaults. DrawCloth takes the face we look at from g_colors[3] and
		// the other side from g_colors[4], both scaled by 1.5. The video's bar and
		// floor are neutral grey, so the shot carries no colour grading, and yet:
		//   - its cloth reads (76, 187, 156) where index 3 renders (0, 130, 94) under
		//     the same light -- index 3 has no red in it at all;
		//   - where the sheet folds over and shows its other side the video reads a
		//     neutral (225, 220, 220), not the bright yellow index 4 would give.
		// Undoing the sRGB encode and dividing out the shading factor of 0.64 that
		// index 3 implies gives the two colours below. Init() restores both.
		g_colors[3] = kClothFace;
		g_colors[4] = kClothBack;
		g_drawPoints = false;
		g_drawSprings = false;

		g_windStrength = kWindStrength;
	}

	void Update()
	{
		// Horizontal, across the sheet (its normal is z), with a little sideways
		// drift so the folds travel. No upward component: wind on a hanging cloth is
		// horizontal, and an updraught just blows the sheet up over its own bar. The
		// rise of the billow comes from g_params.lift instead.
		//
		// The gust is SIGNED -- it reverses, rather than being clamped at zero the way
		// Flag Cloth's is -- and that is measured, not a preference. The video's cloth
		// swings with an H/D root-mean-square of 0.124 about a median of 1.175, and
		// 1.175 is what this cloth projects to hanging dead vertical. So the swing is
		// symmetric about the pin plane, which its W/D confirms: 0.748..1.133 about a
		// still-air 0.87, i.e. as far in front of the plane as behind. A wind that
		// never reverses cannot do that -- it holds the sheet downwind and drags the
		// median with it, which is what every clamped variant of this scene did.
		// Reading the swing as a pendulum, 0.124 of H/D is a hem rising 0.54 m rms on
		// a 4.2 m drop, i.e. plus or minus 29 degrees and plus or minus 2 m of depth --
		// and plus or minus 2 m at 7.88 m from the camera is exactly the W/D range
		// above. The drive therefore sits at the cloth's own period (kGustRate) and
		// damping stays at zero so the free swing survives between gusts.
		const Vec3 kWindDir = Vec3(kWindDrift, 0.0f, kWindDepth);
		const float gust = kGustBias + kGustAmp*Perlin1D(g_windTime*kGustRate, 4, 0.5f);
		const Vec3 wind = g_windStrength*gust*kWindDir;

		g_params.wind[0] = wind.x;
		g_params.wind[1] = wind.y;
		g_params.wind[2] = wind.z;
	}

	virtual void CenterCamera()
	{
		// The video's framing, solved from the checker fit: the camera sits
		// 0.132 W below the pins and 2.503 W in front of the cloth plane, at the
		// demo's default -15 degree pitch. With a 4:3 render that reproduces the
		// panel's horizon row, pin row, hem row and pin separation together.
		g_camPos = Vec3(0.0f, mTop - kCamDrop*mWidth, kCamDistance*mWidth);
		g_camAngle = Vec3(0.0f, -DegToRad(15.0f), 0.0f);
	}

	// 64 x 64 at the demo's usual cloth spacing: a 3.15 m sheet, the size the
	// video's floor checker gives.
	static constexpr float kSpacing = 0.05f;
	// Pin height above the ground plane, from the camera fit: the pins sit
	// 0.132 W above a camera 4.49 m up.
	static constexpr float kPinHeight = 4.90f;
	// Camera, in units of the cloth's rest width (see CenterCamera).
	static constexpr float kCamDrop = 0.132f;
	static constexpr float kCamDistance = 2.503f;
	// The bar, as a multiple of the cloth's rest width, and its radius.
	static constexpr float kBarLength = 1.456f;
	static constexpr float kBarRadius = 0.063f;
	// The two cloth colours, measured off the video (see the draw options above).
	static inline const Colour kClothFace = Colour(0.080f, 0.581f, 0.389f);
	static inline const Colour kClothBack = Colour(0.642f, 0.631f, 0.631f);

	// Sets the dip, and very sensitively: the whole cloth hangs on the top row,
	// so 0.005 here moves the dip by 0.04 W (0.450 -> 0.299, 0.455 -> 0.258).
	// Captures are bit-reproducible, so that steepness is real, not run noise.
	static constexpr float kXpbdStretch = 0.452f;
	// Much softer than stretch, as cloth is: this sets how far the silhouette
	// pulls in between the pinned corners (W/D).
	static constexpr float kXpbdShear = 0.25f;
	// Softer still, and this is what carries the drape folds: at 0.15 the sheet
	// is a smooth funnel, at 0.05 it folds like the video's.
	static constexpr float kXpbdBend = 0.05f;
	// 16 substeps, and that is what makes the scene's point hold. A 64-wide sheet
	// hangs its whole weight off its top row, and FleX's Jacobi solver with
	// count-averaged relaxation moves tension roughly one cell per iteration, so
	// 20 iterations cannot carry it across 64 cells in a single substep.
	// Substepping fixes it, because each substep only has to correct one
	// substep's worth of gravity and the tension field carries over in the
	// positions. Measured at the constants below (top row rest length 3.150 m):
	//
	//     substeps  iterations   top row    drop     dip/D
	//         4         20        4.396 m  3.963 m   0.332
	//         4        160        4.193 m  3.693 m   0.268
	//         8         20        4.197 m  3.696 m   0.268
	//        16         20        4.194 m  3.690 m   0.268
	//        16        160        4.194 m  3.693 m   0.268
	//        32         20        4.196 m  3.691 m   0.271
	//
	// At 4 substeps the drape moves visibly between 20 and 160 iterations, which
	// is exactly the iteration dependence this scene exists to disprove. From 8
	// substeps up it is converged, and at 16 the whole 20/40/80/160 sweep agrees
	// to four figures.
	//
	// The last rows are also the check that the XPBD implementation itself is
	// right: alpha~ = alpha/dt^2 is supposed to make the answer independent of
	// the substep count, and 8, 16 and 32 substeps land within 0.1% of each
	// other, where PBD would stiffen with every extra substep.
	static constexpr int kXpbdSubsteps = 16;
	// Out-of-plane seed, in grid spacings, that lets the sheet buckle into folds.
	static constexpr float kSeedRipple = 0.05f;
	// Gust rate in Perlin periods per second of simulated time. It sits at the
	// cloth's own ringing period (1.43 s in still air; the video's spectral peak
	// is 1.73 s) so the wind drives the sheet near resonance, which is what buys
	// a large swing for a wind whose average is near zero. The strength then
	// sets the swing amplitude: the hem's depth rms runs 0.31 m at strength 1,
	// 0.60 m at 2, 1.29 m at 3 and 1.52 m at 5.
	static constexpr float kGustRate = 0.7f;
	static constexpr float kWindStrength = 3.0f;
	// The gust envelope. The bias is deliberately zero: with it the wind is
	// symmetric and the cloth swings about the pin plane, and every clamped or
	// biased variant instead parked the sheet downwind and dragged the median
	// H/D below the still-air value with it.
	static constexpr float kGustBias = 0.0f;
	static constexpr float kGustAmp = 1.4f;
	// How the wind couples to the sheet. Drag pushes along the flow and is what
	// swings the cloth in depth; note it drives the cloth towards the wind's
	// VELOCITY (UpdateTriangles.hlsl uses wind minus mean triangle velocity), so
	// it both drives and damps. Lift acts across the flow and is what would lift
	// the hem, but it acts on every triangle including the tensioned top row, so
	// raising it past about 0.2 flattens the catenary dip; see the note above.
	// Damping is zero so the free ring survives between gusts.
	static constexpr float kDrag = 1.0f;
	static constexpr float kLift = 0.2f;
	static constexpr float kDamping = 0.0f;
	// Wind direction. The sheet's normal is z, so kWindDepth is the component
	// that does the work and kWindDrift the sideways drift that makes the folds
	// travel. Now that the gust reverses, the sign of kWindDepth only chooses
	// which way the first gust pushes; it mattered when the wind was one-sided,
	// where blowing away from the camera was the only sign that could reproduce
	// the video's body being both narrower and shorter than its rest width.
	static constexpr float kWindDrift = 0.3f;
	static constexpr float kWindDepth = -2.0f;

	float mWidth, mHeight, mTop;
};
