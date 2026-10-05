# flex_demo: the FleX demo's scenes on the engine (FleX/demo/scenes), one executable with the demo's
# scene registry. Steps 10a and 10b of the migration port one or more scenes per feature family: fluid (DamBreak,
# Fluid Block, Buoyancy, Surface Tension, Viscosity, Adhesion), granular, cloth (Flag Cloth, Tearing), rigid bodies,
# soft and plastic bodies, inflatables, triangle mesh and distance field collision, ropes (Pasta), force fields,
# moving frames (Local Space Cloth and Fluid), rigid-fluid coupling and the four XPBD reference scenes; scenes/ has
# one header per FleX scene file or family.
ethereal_phys_add_sample(flex_demo
	SOURCES
		main.cpp
		scenes/collision.h
		scenes/dam_break.h
		scenes/flag_cloth.h
		scenes/fluid_block.h
		scenes/fluids.h
		scenes/force_field.h
		scenes/granular_pile.h
		scenes/inflatable.h
		scenes/local_space.h
		scenes/rigid_pile.h
		scenes/scene_common.h
		scenes/soft_body.h
		scenes/tearing.h
		scenes/xpbd.h
	SMOKE_FRAMES 10)
