#include "demo_app_impl.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>

// The FleX demo's DoUI (reference/flex/demo/main.cpp) with Dear ImGui (Donut's ImGui pass) in place of the demo's
// own immediate-mode UI: the statistics in the top right, the scene list and the options panel on the left, the
// same entries and ranges.

namespace ephys::framework {

int DemoApp::DoUI() {
    if (!m_ShowHelp || !m_Physics)
        return -1;

    const int numParticles = int(m_Physics->GetActiveCount());
    const ImGuiIO& io = ImGui::GetIO();

    // ---- statistics ----
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 10.0f, 10.0f), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.35f);
    if (ImGui::Begin("Statistics", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                         ImGuiWindowFlags_NoNav)) {
        const helpers::SimBuffers& b = g.buffers;
        ImGui::Text("Frame: %d", g.frame);
        ImGui::Text("Frame Time: %.2fms", m_RealDt * 1000.0f);
        ImGui::Text("Sim Time (CPU): %.2fms", m_UpdateTime * 1000.0f);
        ImGui::TextColored(ImVec4(0.97f, 0.59f, 0.27f, 1.0f), "Sim Latency (GPU): %.2fms",
                           m_Profile ? m_SimLatency : 0.0f);
        ImGui::Separator();
        ImGui::Text("Particle Count: %d", numParticles);
        ImGui::Text("Diffuse Count: %u", m_Physics->GetDiffuseCount());
        ImGui::Text("Rigid Count: %d", b.rigidOffsets.empty() ? 0 : int(b.rigidOffsets.size()) - 1);
        ImGui::Text("Spring Count: %d", int(b.springLengths.size()));
        ImGui::Text("Num Substeps: %d", g.numSubsteps);
        ImGui::Text("Num Iterations: %d", g.params.numIterations);
        ImGui::Text("Solver: %s", g.solverMode == SolverMode::XPBD ? "XPBD" : "PBD");
        ImGui::Text("Render arrays: %s", m_Physics->AreRenderArraysShared() ? "zero copy" : "copied (D3D11)");
        ImGui::Separator();
        ImGui::Text("Device: %s", m_DeviceName.c_str());

        if (m_Profile && m_Physics->GetTimers()) {
            PhysTimers t;
            m_Physics->GetTimers()->getTimers(t);
            const float ms = 1000.0f;
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.97f, 0.59f, 0.27f, 1.0f), "Total GPU Sim Latency: %.2fms", t.total * ms);
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "GPU Latencies");
            ImGui::Text("Predict: %.2fms", t.predict * ms);
            ImGui::Text("Create Cell Indices: %.2fms", t.createCellIndices * ms);
            ImGui::Text("Sort Cell Indices: %.2fms", t.sortCellIndices * ms);
            ImGui::Text("Reorder: %.2fms", t.reorder * ms);
            ImGui::Text("CreateGrid: %.2fms", t.createGrid * ms);
            ImGui::Text("Collide Particles: %.2fms", t.collideParticles * ms);
            ImGui::Text("Collide Shapes: %.2fms", t.collideShapes * ms);
            ImGui::Text("Collide Triangles: %.2fms", t.collideTriangles * ms);
            ImGui::Text("Calculate Density: %.2fms", t.calculateDensity * ms);
            ImGui::Text("Solve Densities: %.2fms", t.solveDensities * ms);
            ImGui::Text("Solve Velocities: %.2fms", t.solveVelocities * ms);
            ImGui::Text("Solve Rigids: %.2fms", t.solveShapes * ms);
            ImGui::Text("Solve Springs: %.2fms", t.solveSprings * ms);
            ImGui::Text("Solve Inflatables: %.2fms", t.solveInflatables * ms);
            ImGui::Text("Solve Contacts: %.2fms", t.solveContacts * ms);
            ImGui::Text("Apply Deltas: %.2fms", t.applyDeltas * ms);
            ImGui::Text("Finalize: %.2fms", t.finalize * ms);
            ImGui::Text("Update Triangles: %.2fms", t.updateTriangles * ms);
            ImGui::Text("Update Normals: %.2fms", t.updateNormals * ms);
            ImGui::Text("Update Bounds: %.2fms", t.updateBounds * ms);
            ImGui::Text("Calculate Anisotropy: %.2fms", t.calculateAnisotropy * ms);
            ImGui::Text("Update Diffuse: %.2fms", t.updateDiffuse * ms);
        }
    }
    ImGui::End();

    // ---- the scenes ----
    int newScene = -1;
    const float uiBorder = 20.0f;
    const float uiWidth = 250.0f;
    const float uiOffset = 250.0f;
    const float sceneHeight = m_TweakPanel ? uiOffset : io.DisplaySize.y - 2.0f * uiBorder;
    ImGui::SetNextWindowPos(ImVec2(uiBorder, uiBorder), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(uiWidth, sceneHeight), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Scene")) {
        for (int i = 0; i < int(m_Desc.scenes.size()); ++i) {
            const bool current = i == m_Scene;
            if (current)
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 151.0f / 255.0f, 61.0f / 255.0f, 1.0f));
            if (ImGui::Selectable(m_Desc.scenes[size_t(i)]->GetName(), i == m_SelectedScene))
                newScene = i;
            if (current)
                ImGui::PopStyleColor();
        }
    }
    ImGui::End();

    // ---- the options ----
    if (m_TweakPanel) {
        ImGui::SetNextWindowPos(ImVec2(uiBorder, uiBorder * 2.0f + uiOffset), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(uiWidth + 70.0f, io.DisplaySize.y - uiOffset - uiBorder * 3.0f),
                                 ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Options")) {
            PhysParams& p = g.params;

            ImGui::TextUnformatted("Global");
            ImGui::Checkbox("Emit particles", &g.emit);
            ImGui::Checkbox("Pause", &g.pause);
            if (ImGui::Button("Step"))
                g.step = true;
            ImGui::Checkbox("Profile (GPU timers)", &m_Profile);
            ImGui::Separator();

            ImGui::Checkbox("Wireframe", &g.wireframe);
            ImGui::Checkbox("Draw Points", &g.drawPoints);
            ImGui::Checkbox("Draw Fluid", &g.drawEllipsoids);
            if (ImGui::Checkbox("Draw Mesh", &g.drawMesh))
                g.drawRopes = !g.drawRopes;
            ImGui::Checkbox("Draw Diffuse", &g.drawDiffuse);
            ImGui::Checkbox("Draw Basis", &g.drawBases);
            bool springs = g.drawSprings != 0;
            if (ImGui::Checkbox("Draw Springs", &springs))
                g.drawSprings = springs ? 1 : 0;
            ImGui::Checkbox("Draw Normals", &g.drawNormals);
            ImGui::Separator();

            // the scene's options
            m_Desc.scenes[size_t(m_Scene)]->DoGui(g);

            if (ImGui::Button("Reset Scene"))
                m_ResetScene = true;

            ImGui::Separator();
            ImGui::SliderInt("Num Substeps", &g.numSubsteps, 1, 32);
            ImGui::SliderInt("Num Iterations", &p.numIterations, 1, 160);

            // XPBD (Macklin et al. 2016): FleX switched the formulation in place; here a solver is created for one
            // formulation (SolverDesc::mode), so a change restarts the scene with the other one
            ImGui::Separator();
            int solver = g.solverMode == SolverMode::XPBD ? 1 : 0;
            if (ImGui::Combo("Solver", &solver, "PBD\0XPBD\0")) {
                m_Options.solverMode = solver;
                m_ResetScene = true;
            }
            // the stiffness range the [0,1] coefficients map onto, edited in log10 (N/m); the solver's
            // IPhysSolverXPBD::setCompliance takes a change at the next step
            float n = log10f(std::max(g.xpbd.stiffnessMin, 1.0e-3f));
            if (ImGui::SliderFloat("Stiffness Min (log10)", &n, -3.0f, 12.0f)) {
                g.xpbd.stiffnessMin = powf(10.0f, n);
                g.xpbd.stiffnessMax = std::max(g.xpbd.stiffnessMax, g.xpbd.stiffnessMin);
            }
            n = log10f(std::max(g.xpbd.stiffnessMax, 1.0e-3f));
            if (ImGui::SliderFloat("Stiffness Max (log10)", &n, -3.0f, 12.0f)) {
                g.xpbd.stiffnessMax = powf(10.0f, n);
                g.xpbd.stiffnessMin = std::min(g.xpbd.stiffnessMin, g.xpbd.stiffnessMax);
            }
            ImGui::SliderFloat("Spring Damping", &g.xpbd.springDamping, 0.0f, 100.0f);
            ImGui::SliderFloat("Volume Compliance", &g.xpbd.volumeCompliance, 0.0f, 0.05f, "%.4f");

            ImGui::Separator();
            ImGui::SliderFloat("Gravity X", &p.gravity[0], -50.0f, 50.0f);
            ImGui::SliderFloat("Gravity Y", &p.gravity[1], -50.0f, 50.0f);
            ImGui::SliderFloat("Gravity Z", &p.gravity[2], -50.0f, 50.0f);

            ImGui::Separator();
            ImGui::SliderFloat("Radius", &p.radius, 0.01f, 0.5f);
            ImGui::SliderFloat("Solid Radius", &p.solidRestDistance, 0.0f, 0.5f);
            ImGui::SliderFloat("Fluid Radius", &p.fluidRestDistance, 0.0f, 0.5f);

            // common
            ImGui::Separator();
            ImGui::SliderFloat("Dynamic Friction", &p.dynamicFriction, 0.0f, 1.0f);
            ImGui::SliderFloat("Static Friction", &p.staticFriction, 0.0f, 1.0f);
            ImGui::SliderFloat("Particle Friction", &p.particleFriction, 0.0f, 1.0f);
            ImGui::SliderFloat("Restitution", &p.restitution, 0.0f, 1.0f);
            ImGui::SliderFloat("SleepThreshold", &p.sleepThreshold, 0.0f, 1.0f);
            ImGui::SliderFloat("Shock Propagation", &p.shockPropagation, 0.0f, 10.0f);
            ImGui::SliderFloat("Damping", &p.damping, 0.0f, 10.0f);
            ImGui::SliderFloat("Dissipation", &p.dissipation, 0.0f, 0.01f, "%.4f");
            ImGui::SliderFloat("SOR", &p.relaxationFactor, 0.0f, 5.0f);

            ImGui::SliderFloat("Collision Distance", &p.collisionDistance, 0.0f, 0.5f);
            ImGui::SliderFloat("Collision Margin", &p.shapeCollisionMargin, 0.0f, 5.0f);

            // cloth
            ImGui::Separator();
            ImGui::SliderFloat("Wind", &g.windStrength, -1.0f, 1.0f);
            ImGui::SliderFloat("Drag", &p.drag, 0.0f, 1.0f);
            ImGui::SliderFloat("Lift", &p.lift, 0.0f, 1.0f);

            // fluid
            ImGui::Separator();
            ImGui::SliderFloat("Adhesion", &p.adhesion, 0.0f, 10.0f);
            ImGui::SliderFloat("Cohesion", &p.cohesion, 0.0f, 0.2f, "%.4f");
            ImGui::SliderFloat("Surface Tension", &p.surfaceTension, 0.0f, 50.0f);
            ImGui::SliderFloat("Viscosity", &p.viscosity, 0.0f, 120.0f);
            ImGui::SliderFloat("Vorticity Confinement", &p.vorticityConfinement, 0.0f, 120.0f);
            ImGui::SliderFloat("Solid Pressure", &p.solidPressure, 0.0f, 1.0f);
            ImGui::SliderFloat("Surface Drag", &p.freeSurfaceDrag, 0.0f, 1.0f);
            ImGui::SliderFloat("Buoyancy", &p.buoyancy, -1.0f, 1.0f);

            ImGui::Separator();
            ImGui::SliderFloat("Anisotropy Scale", &p.anisotropyScale, 0.0f, 30.0f);
            ImGui::SliderFloat("Smoothing", &p.smoothing, 0.0f, 1.0f);

            // diffuse
            ImGui::Separator();
            ImGui::SliderFloat("Diffuse Threshold", &p.diffuseThreshold, 0.0f, 1000.0f);
            ImGui::SliderFloat("Diffuse Buoyancy", &p.diffuseBuoyancy, 0.0f, 2.0f);
            ImGui::SliderFloat("Diffuse Drag", &p.diffuseDrag, 0.0f, 2.0f);
            ImGui::SliderFloat("Diffuse Scale", &g.diffuseScale, 0.0f, 1.5f);
            ImGui::SliderFloat("Diffuse Alpha", &g.diffuseColor.w, 0.0f, 3.0f);
            ImGui::SliderFloat("Diffuse Inscatter", &g.diffuseInscatter, 0.0f, 2.0f);
            ImGui::SliderFloat("Diffuse Outscatter", &g.diffuseOutscatter, 0.0f, 2.0f);
            ImGui::SliderFloat("Diffuse Motion Blur", &g.diffuseMotionScale, 0.0f, 5.0f);
            ImGui::SliderInt("Diffuse Ballistic", &p.diffuseBallistic, 1, 40);
        }
        ImGui::End();
    }

    if (newScene != -1) {
        m_SelectedScene = newScene;
        m_NewScene = newScene;
    }
    return newScene;
}

}  // namespace ephys::framework
