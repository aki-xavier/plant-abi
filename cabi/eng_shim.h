// eng_shim.h — minimal C ABI between the Rust control core and the physics engine: the model, the state
// layout and the contact readout are the whole contract. This header names no engine: cabi/mj_shim.c is
// one implementation of it, and another engine's would implement these same names.
// Functions returning int yield 0 on success and -1 on failure; after a -1, eng_robot_last_error()
// describes the cause (also mirrored to stderr).

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void* eng_context_open(void);
void eng_context_close(void* ctx);

void* eng_scene_open(void* ctx, const char* name);
void eng_scene_free(void* ctx, void* scene);

// Loads a URDF robot, welds the injected world joint to the world and creates the bot in scene (fills *out_robot, or -1 with a message in err).
int eng_robot_attach(void* scene, const char* urdf_path, const char* ee_link, void** out_robot, char* err, int err_cap);

// As eng_robot_attach, but the world joint stays Free: the base owns 6 DOFs ([x y z rotvec] leading every
// pose/velocity/force vector).
int eng_robot_attach_floating(void* scene, const char* urdf_path, const char* ee_link, void** out_robot, char* err, int err_cap);

// Explicit-parameter variant: weld_root 0 = floating base; voxel_meters is the replaced engine's SDF voxel size, ignored here (meshes collide directly), pass 0.
int eng_robot_attach_ex(void* scene, const char* urdf_path, const char* ee_link, int weld_root, double voxel_meters, void** out_robot, char* err, int err_cap);

// Adds a static rigid box whose shape loads from shape_path (.stl/.obj/.ply mesh), centered at world (px, py, pz); one wall per scene, later calls are no-ops.
int eng_scene_add_box(void* scene, const char* shape_path, float px, float py, float pz);

// Adds an infinite static ground plane (normal . x = distance) as an analytic collider — the ground for legged robots.
int eng_scene_add_plane(void* scene, float nx, float ny, float nz, float distance);

// ---- rendering (an offscreen frame) -----------------------------------------

// Draws one frame and fills rgb with packed RGB8 (width*height*3, top row first); a null lookat follows the robot's CoM, azimuth/elevation in degrees, distance in metres (<= 0 takes the extent).
int eng_scene_render(void* scene, int width, int height, const float* lookat, float distance,
                    float azimuth, float elevation, unsigned char* rgb, int rgb_cap);

const char* eng_scene_render_error(void* scene);

// Retunes link-against-static contact: penalty stiffness [Pa/m], normal viscous damping, Coulomb friction (0 leaves a field untouched).
int eng_robot_set_contact_params(void* robot, double penalty, double damping, double coulomb);

// States the contact REPORT the caller wants: 1 = the penalty report at `stiffness` N/m, 0 = the solver's own per-contact table. Before any call, an implementation-defined default applies — an implementation may take it from its environment, and naming it is the implementation's business; stating the pair here overrides it for this robot.
int eng_robot_set_contact_report(void* robot, int penalty, double stiffness);

// Aggregate contact force and torque over the robot's own bodies, world frame (N, N.m), as the solver applied them — NOT the per-link tables' reconstruction.
int eng_robot_contact_force(void* robot, float* out_force);
int eng_robot_contact_torque(void* robot, float* out_torque);

// Per-link aggregate contact forces, world frame: 3 * nlinks floats, one Real3 per slot, slot s = link
// s+1 in chain order and the LAST slot the floating root; returns the link count or -1. n_links_max is a
// SLOT count while the return is a LINK count, a short buffer truncates silently, and a call advances the
// readout's low-pass: read the tables once per step.
int eng_robot_link_contact_forces(void* robot, float* out, int n_links_max);

// Per-link aggregate contact WRENCH, world frame, about each link's own origin: 6 * nlinks floats,
// [fx fy fz tx ty tz] per slot, the same layout and truncation/filter contract as the forces above. The
// per-foot pressure-distribution sense: for F = (0, 0, fz) acting at (dx, dy, dz) from that origin,
// M = r x F, so the pressure point is cop_x = -ty/fz, cop_y = +tx/fz.
int eng_robot_link_contact_wrenches(void* robot, float* out, int n_links_max);
void eng_robot_free(void* robot);

const char* eng_robot_last_error(void* robot);

int eng_robot_num_dofs(void* robot);
int eng_robot_set_state(void* robot, const float* q, const float* v);
int eng_robot_step(void* robot, const float* tau, int n_sub, double dt);
int eng_robot_get_q(void* robot, float* q_out);
int eng_robot_get_v(void* robot, float* v_out);
int eng_robot_ee_pos(void* robot, float* pos_out);
int eng_robot_enable_newton_euler(void* robot, int enable);
// Zero Coulomb/stiction friction, keep the URDF viscous damping.
int eng_robot_keep_viscous_only(void* robot);

#ifdef __cplusplus
}
#endif
