# flex_demo: the FleX demo's scenes on the engine (reference/flex/demo/scenes), one executable with the demo's
# scene registry. Step 10a of the migration ports the first scenes, one or more per feature family the framework
# draws: fluid (DamBreak, Fluid Block: the screen-space fluid surface and diffuse particles), granular (Granular
# Pile), cloth (Flag Cloth) and rigid bodies (Rigid2/4/8); scenes/ has one header per FleX scene file.
ethereal_phys_add_sample(flex_demo
	SOURCES
		main.cpp
		scenes/dam_break.h
		scenes/flag_cloth.h
		scenes/fluid_block.h
		scenes/granular_pile.h
		scenes/rigid_pile.h
		scenes/scene_common.h
	SMOKE_FRAMES 10)
