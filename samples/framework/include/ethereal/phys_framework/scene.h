#ifndef ETHEREAL_PHYS_FRAMEWORK_SCENE_H
#define ETHEREAL_PHYS_FRAMEWORK_SCENE_H

// Scene: one entry of a sample's scene registry (the FleX demo's Scene, FleX/demo/scenes.h).
//
// Part of ethereal_phys_framework (samples/framework). The hooks are FleX's, with the demo's globals passed in as
// one DemoState (demo_state.h):
//
//   Initialize       author the scene: particles, constraints and shapes into g.buffers (ethereal_phys_helpers),
//                    parameters into g.params, draw options; called after the framework reset g to FleX's defaults
//   PostInitialize   after the domain, the scene graph and the solver exist
//   Update           every simulated frame, before the frame is drawn and stepped; call g.MapBuffers() before
//                    reading or writing the particle arrays of g.buffers (FleX's were always mapped here)
//   Sync             every frame, after the particles were sent and before the step: ask for the upload of the
//                    constraints the scene changed (g.SetSprings(), g.SetDynamicTriangles(), ...; FleX's NvFlexSet*)
//   Draw             custom drawing through g.context (pass 1: the shadow map, pass 0: the lit scene)
//   KeyDown, DoGui   input and the scene's part of the ImGui panel
//   CenterCamera     move g.camPos / g.camAngle after the framework centred the camera on the scene
//
// FleX's GetBasis (the frame of its Player scene, which is not ported) is not here; the local-space scenes use
// the engine's IMovingFrame (ext/moving_frame.h) through g.phys.

#include <ethereal/phys_framework/demo_state.h>

namespace ephys::framework {

class Scene {
 public:
    explicit Scene(const char* name) : mName(name) {}
    virtual ~Scene() = default;

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

    virtual void Initialize(DemoState& g) = 0;
    virtual void PostInitialize(DemoState& g) { (void)g; }
    virtual void Update(DemoState& g) { (void)g; }
    virtual void Sync(DemoState& g) { (void)g; }
    virtual void Draw(DemoState& g, int pass) {
        (void)g;
        (void)pass;
    }
    // key: the character (lower case for letters), as FleX's KeyDown
    virtual void KeyDown(DemoState& g, int key) {
        (void)g;
        (void)key;
    }
    virtual void DoGui(DemoState& g) { (void)g; }
    virtual void CenterCamera(DemoState& g) { (void)g; }

    [[nodiscard]] const char* GetName() const { return mName; }

 private:
    const char* mName;
};

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_SCENE_H */
