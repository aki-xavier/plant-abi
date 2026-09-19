// mj_shim_smoke.c — drives the eng_* ABI the way a caller does, so the MuJoCo backend can be checked on
// its own: context, scene, ground, attach, state, step, per-link contact readout. It also pins that a part
// which is not ported fails with a message rather than a plausible zero.
//
// The second half covers the entry points this file did not drive (see the block before the frees):
// eng_robot_attach_ex, eng_scene_add_box, eng_robot_set_contact_params, eng_robot_set_contact_report,
// eng_robot_keep_viscous_only and eng_robot_last_error. Each is pinned by what its own contract can carry
// — a returned status, the reading a stated stiffness produces, what "once per scene" looks like from
// outside — and the two friction writers are pinned as statuses only: their EFFECT is a contact
// phenomenon, and the single patch this scenario rests on rocks and bounces under it (measured), so
// pinning it here would pin the rock, not the friction. That belongs to a caller's own scenario tests.
//
// The model it drives is ../control-model's, not this crate's, so the models DIRECTORY is an argument
// (default ".") — the same reason mj_model_load takes one: a self-check that hardcoded a neighbour's
// relative path broke silently the day those files moved out of the repository it was run from.
//
// Build: cabi/build_mj.sh then cc -O2 -Icabi -o $BUILD/mj_shim_smoke cabi/mj_shim_smoke.c
//        -L$BUILD/bin -leng_shim -Wl,-rpath,$BUILD/bin -Wl,-rpath,$MJ
// Run:   mj_shim_smoke <models-dir>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "eng_shim.h"

static int failures = 0;

static void check(int cond, const char* what) {
  printf("%s %s\n", cond ? "ok  " : "FAIL", what);
  if (!cond) {
    failures++;
  }
}

// the per-link readout's total along z, and its total of absolute components: the first is the load a
// resting contact carries (sign and all), the second is "is anything touching at all"
static void contact_totals(void* robot, float* fz, float* mag) {
  static float lf[3 * 64];
  int n = eng_robot_link_contact_forces(robot, lf, 64);
  float z = 0.0f;
  float m = 0.0f;
  for (int b = 0; b < n && b < 64; ++b) {
    z += lf[3 * b + 2];
    m += fabsf(lf[3 * b + 0]) + fabsf(lf[3 * b + 1]) + fabsf(lf[3 * b + 2]);
  }
  *fz = z;
  *mag = m;
}

// stepn advances a robot by n substeps of dt with no torque, which is the state every contact reading
// below is taken in
static void stepn(void* robot, int n, double dt) {
  static float tau[64];
  for (int i = 0; i < n; ++i) {
    eng_robot_step(robot, tau, 1, dt);
  }
}

int main(int argc, char** argv) {
  setvbuf(stdout, NULL, _IONBF, 0);  // a crash must not eat the progress report
  const char* root = (argc > 1) ? argv[1] : ".";
  static char urdf[1024];
  snprintf(urdf, sizeof(urdf), "%s/z1/z1.urdf", root);
  static char wall_path[1024];
  snprintf(wall_path, sizeof(wall_path), "%s/wall.stl", root);

  void* ctx = eng_context_open();
  check(ctx != NULL, "context opens");

  void* scene = eng_scene_open(ctx, "mj_smoke");
  check(scene != NULL, "scene opens");

  check(eng_scene_add_plane(scene, 0.0f, 0.0f, 1.0f, 0.0f) == 0, "ground plane added");

  // a scene holds one robot, so the floating case gets its own
  void* scene2 = eng_scene_open(ctx, "mj_smoke_float");
  check(scene2 != NULL, "a second scene opens");
  check(eng_scene_add_plane(scene2, 0.0f, 0.0f, 1.0f, 0.0f) == 0, "its ground plane is added");

  char err[256] = {0};
  void* robot = NULL;
  int rc = eng_robot_attach(scene, urdf, "link06", &robot, err, sizeof(err));
  printf("     attach rc=%d err='%s'\n", rc, err);
  check(rc == 0 && robot != NULL, "arm attaches from its URDF");

  int nv = eng_robot_num_dofs(robot);
  printf("     nv=%d\n", nv);
  check(nv == 6, "the z1 has six driven joints");

  float q[16] = {0};
  float v[16] = {0};
  float tau[16] = {0};
  tau[0] = 1.0f;
  check(eng_robot_set_state(robot, q, v) == 0, "state installs");

  // a torque's own effect, isolated from gravity: the same state stepped with and without it
  float q_a[16] = {0};
  float q_b[16] = {0};
  float v_a[16] = {0};
  float v_b[16] = {0};
  float tau_off[16] = {0};
  eng_robot_set_state(robot, q, v);
  check(eng_robot_step(robot, tau_off, 1, 1.0e-3) == 0, "step runs");
  eng_robot_get_q(robot, q_a);
  eng_robot_get_v(robot, v_a);
  eng_robot_set_state(robot, q, v);
  check(eng_robot_step(robot, tau, 1, 1.0e-3) == 0, "step with a torque runs");
  eng_robot_get_q(robot, q_b);
  eng_robot_get_v(robot, v_b);
  printf("     v1: no torque=%.6f with torque=%.6f  dq1=%.8f\n", v_a[0], v_b[0], q_b[0] - q_a[0]);
  check(v_b[0] > v_a[0], "a positive torque adds velocity the joint's way");

  float q_out[16] = {0};
  float v_out[16] = {0};
  check(eng_robot_get_q(robot, q_out) == 0, "state reads back");
  check(eng_robot_get_v(robot, v_out) == 0, "velocity reads back");

  float pos[3] = {0};
  check(eng_robot_ee_pos(robot, pos) == 0, "end-effector position reads");
  printf("     ee=[%.4f %.4f %.4f]\n", pos[0], pos[1], pos[2]);

  static float lf[3 * 64];
  int nlinks = eng_robot_link_contact_forces(robot, lf, 64);
  printf("     link_contact_forces -> nlinks=%d\n", nlinks);
  check(nlinks > 0, "per-link contact forces report the link count");

  // MuJoCo's own renderer, offscreen: the check is that a frame comes back and is not one flat colour
  {
    enum { RW = 320, RH = 240 };
    static unsigned char frame[RW * RH * 3];
    int rc = eng_scene_render(scene, RW, RH, NULL, 0.0f, 90.0f, -20.0f, frame, sizeof(frame));
    printf("     render rc=%d (%s)\n", rc, rc == 0 ? "ok" : eng_scene_render_error(scene));
    check(rc == 0, "a frame renders");
    if (rc == 0) {
      unsigned char lo = 255;
      unsigned char hi = 0;
      long sum = 0;
      for (int i = 0; i < RW * RH * 3; ++i) {
        if (frame[i] < lo) lo = frame[i];
        if (frame[i] > hi) hi = frame[i];
        sum += frame[i];
      }
      printf("     frame: %dx%d, luminance %d..%d, mean %ld\n", RW, RH, (int)lo, (int)hi,
             sum / (RW * RH * 3));
      // not just "more than one colour": a ruined extent still shows a shaded ground, so require a lit surface
      check(hi > 60, "the frame has a lit surface in it, not just a shaded ground");
    }
  }

  static float lw[6 * 64];
  int nlinks_w = eng_robot_link_contact_wrenches(robot, lw, 64);
  check(nlinks_w > 0, "per-link contact wrenches report the link count");

  // the floating base: the same URDF with a free joint on its root — the state layout becomes [x y z
  // rotvec, joints] and the DOF count grows by six (the arm's model stands in for any floating base)
  void* frobot = NULL;
  char ferr[256] = {0};
  int frc = eng_robot_attach_floating(scene2, urdf, "link06", &frobot, ferr, sizeof(ferr));
  printf("     floating attach rc=%d err='%s'\n", frc, ferr);
  check(frc == 0 && frobot != NULL, "a floating base attaches");
  int fnv = eng_robot_num_dofs(frobot);
  printf("     floating num_dofs=%d\n", fnv);
  check(fnv == 12, "six base DOFs plus the arm's six");

  // a base state the ABI's layout can carry (a position, a NON-trivial rotation vector, a world-frame omega): reading both back tests the conversions in both directions
  float fq[16] = {0};
  float fv[16] = {0};
  fq[0] = 0.1f;
  fq[1] = 0.2f;
  fq[2] = 0.5f;
  fq[3] = 0.0f;
  fq[4] = 0.0f;
  fq[5] = 0.5f;  // ~28.6 deg about z
  fv[0] = 0.3f;
  fv[3] = 0.0f;
  fv[4] = 0.0f;
  fv[5] = 1.0f;  // a world-frame spin about z
  check(eng_robot_set_state(frobot, fq, fv) == 0, "floating state installs");
  float fq_back[16] = {0};
  float fv_back[16] = {0};
  eng_robot_get_q(frobot, fq_back);
  eng_robot_get_v(frobot, fv_back);
  printf("     q back=[%.5f %.5f %.5f | %.5f %.5f %.5f]\n", fq_back[0], fq_back[1], fq_back[2],
         fq_back[3], fq_back[4], fq_back[5]);
  printf("     v back=[%.5f %.5f %.5f | %.5f %.5f %.5f]\n", fv_back[0], fv_back[1], fv_back[2],
         fv_back[3], fv_back[4], fv_back[5]);
  check(fabsf(fq_back[0] - fq[0]) < 1e-5f && fabsf(fq_back[2] - fq[2]) < 1e-5f, "base position round-trips");
  check(fabsf(fq_back[5] - fq[5]) < 1e-4f, "the rotation vector round-trips");
  check(fabsf(fv_back[0] - fv[0]) < 1e-5f, "the linear velocity round-trips");
  check(fabsf(fv_back[5] - fv[5]) < 1e-4f, "the world-frame omega round-trips");

  // and it steps: a free base released from rest accelerates at exactly g (a check of the whole base state mapping)
  float ftau[16] = {0};
  float fv_rest[16] = {0};
  fq[2] = 2.0f;  // high enough that no link reaches the ground, and the joints stay at zero because
                 // zero lies inside every limit: a pose outside one makes MuJoCo's limit constraint
                 // yank the joint, and that force — not the engine — is what the base reacts to.
  eng_robot_set_state(frobot, fq, fv_rest);
  check(eng_robot_step(frobot, ftau, 1, 1.0e-3) == 0, "floating step runs");
  eng_robot_get_v(frobot, fv_back);
  printf("     after one free step from rest, v_z=%.6f (expected %.6f)\n", fv_back[2], -9.81e-3);
  printf("     full v back=[");
  for (int i = 0; i < 12; ++i) {
    printf("%.5f%s", fv_back[i], i == 11 ? "]\n" : " ");
  }
  check(fabsf(fv_back[2] + 9.81e-3f) < 2e-4f, "the free base accelerates at g");

  // the other half of the collision-class setup: the base is placed about a millimetre into the ground
  // and only a probe's worth of time is simulated (a deep overlap is turned into a velocity correction).
  fq[2] = 0.090f;
  eng_robot_set_state(frobot, fq, fv_rest);
  for (int i = 0; i < 20; ++i) {
    eng_robot_step(frobot, ftau, 1, 1.0e-5);
  }
  static float lf2[3 * 64];
  eng_robot_link_contact_forces(frobot, lf2, 64);
  float tot_fz = 0.0f;
  for (int b = 0; b < 8; ++b) {
    tot_fz += lf2[3 * b + 2];
  }
  float qz[16] = {0};
  eng_robot_get_q(frobot, qz);
  printf("     after the probe, base z=%.4f (started 0.0900) total contact fz=%.2f N\n", qz[2],
         tot_fz);
  check(tot_fz > 1.0f, "the robot still collides with the scene");

  // ---- the entry points that had no check here -------------
  // Six of the header's functions were never driven from this file, so a header that promised one thing
  // and a shim that did another passed everything. What each one can be pinned by is different, and the
  // difference is the point: a status, a reading, or an absence.

  // eng_robot_attach_ex is the two attach paths behind one door — weld_root 0 is the free base, anything
  // else the welded one — plus the replaced engine's SDF voxel size, which this backend ignores (meshes
  // collide directly). A nonzero voxel is passed on purpose: the two DOF counts below are the statement
  // that the argument is not consulted.
  void* exscene = eng_scene_open(ctx, "mj_smoke_attach_ex");
  check(exscene != NULL, "a scene for attach_ex opens");
  check(eng_scene_add_plane(exscene, 0.0f, 0.0f, 1.0f, 0.0f) == 0, "its ground plane is added");
  char exerr[256] = {0};
  void* exrobot = NULL;
  int exrc = eng_robot_attach_ex(exscene, urdf, "link06", 0, 0.05, &exrobot, exerr, sizeof(exerr));
  printf("     attach_ex(weld_root=0, voxel=0.05) rc=%d err='%s'\n", exrc, exerr);
  check(exrc == 0 && exrobot != NULL, "attach_ex with weld_root 0 attaches");
  int exnv = eng_robot_num_dofs(exrobot);
  printf("     weld_root 0: nv=%d\n", exnv);
  check(exnv == 12, "weld_root 0 is the floating base the voxel size does not change");

  // the report ladder, on a resting contact this scene already holds: the readout's own contract is that
  // eng_robot_set_contact_report STATES what the caller wants, so a tenfold stiffness has to read tenfold,
  // and mode 0 has to give the solver's load back. Both are the header's warning made checkable — the
  // penalty report under-reports a held load, and a caller that states neither keeps the environment's.
  // The base starts about a millimetre into the ground, the one pose this model settles a load in.
  float rq[16] = {0};
  float rv[16] = {0};
  rq[2] = 0.09f;
  eng_robot_set_state(exrobot, rq, rv);
  stepn(exrobot, 200, 1.0e-5);
  float solver_fz = 0.0f;
  float unused_mag = 0.0f;
  contact_totals(exrobot, &solver_fz, &unused_mag);
  const double k_hi = 650000.0;
  const double k_mid = 65000.0;
  const double k_lo = 6500.0;
  float pen_hi = 0.0f;
  float pen_mid = 0.0f;
  float pen_lo = 0.0f;
  eng_robot_set_contact_report(exrobot, 1, k_hi);
  stepn(exrobot, 200, 1.0e-5);
  contact_totals(exrobot, &pen_hi, &unused_mag);
  eng_robot_set_contact_report(exrobot, 1, k_mid);
  stepn(exrobot, 200, 1.0e-5);
  contact_totals(exrobot, &pen_mid, &unused_mag);
  eng_robot_set_contact_report(exrobot, 1, k_lo);
  stepn(exrobot, 200, 1.0e-5);
  contact_totals(exrobot, &pen_lo, &unused_mag);
  eng_robot_set_contact_report(exrobot, 0, 0.0);
  stepn(exrobot, 200, 1.0e-5);
  float solver_again = 0.0f;
  contact_totals(exrobot, &solver_again, &unused_mag);
  printf("     report: solver=%.3f penalty k=%.0f -> %.3f, k=%.0f -> %.3f, k=%.0f -> %.3f, solver "
         "again=%.3f\n",
         solver_fz, k_hi, pen_hi, k_mid, pen_mid, k_lo, pen_lo, solver_again);
  check(pen_hi > 4.0f * pen_mid && pen_mid > 4.0f * pen_lo,
        "the stiffness the caller states is the one the report reads");
  check(pen_lo < solver_fz, "at the lowest stiffness the reported load sits below the solver's own");
  check(solver_again > 4.0f * pen_lo, "mode 0 is the solver's own table again");

  // the two friction writers, as statuses: their effect is a contact phenomenon and this scenario's one
  // patch rocks under it (measured), so what is pinned is which arguments they accept, that a null robot
  // is refused, and that a policy change never costs the caller its handle.
  check(eng_robot_set_contact_params(exrobot, 5e7, 0.0, 1.0) == 0,
        "set_contact_params takes the penalty/damping/Coulomb triple");
  check(eng_robot_set_contact_params(exrobot, 5e7, 0.0, -1.0) == 0,
        "a negative Coulomb is the leave-the-geoms-alone branch");
  check(eng_robot_keep_viscous_only(exrobot) == 0, "keep_viscous_only takes a live robot");
  check(eng_robot_num_dofs(exrobot) == exnv && eng_robot_get_q(exrobot, rq) == 0,
        "and the robot still reads its own state after the policy change");
  check(eng_robot_set_contact_params(NULL, 1.0, 1.0, 1.0) == -1 &&
            eng_robot_set_contact_report(NULL, 1, 1.0) == -1 && eng_robot_keep_viscous_only(NULL) == -1,
        "all three contact calls refuse a null robot");

  // eng_robot_last_error is the other half of every -1 in the ABI: a part with no counterpart here must
  // fail LOUDLY. This is also the one place a live robot can be made to fail on purpose.
  int ne_rc = eng_robot_enable_newton_euler(exrobot, 1);
  const char* ne_msg = eng_robot_last_error(exrobot);
  printf("     enable_newton_euler rc=%d last_error='%s'\n", ne_rc, ne_msg ? ne_msg : "(null)");
  check(ne_rc == -1, "the part this backend has no counterpart for fails rather than faking a zero");
  check(ne_msg != NULL && strstr(ne_msg, "eng_robot_enable_newton_euler") != NULL,
        "and its failure names itself in eng_robot_last_error");
  const char* null_msg = eng_robot_last_error(NULL);
  check(null_msg != NULL && null_msg[0] != '\0', "last_error answers for a null robot too");

  // eng_scene_add_box, in the order a caller reaches it: the wall goes in AFTER the robot exists, which
  // splices it into the live spec and recompiles — the path a construction-time call would never touch.
  // A scene with no ground on purpose: with the plane in, the arm's own resting contact hides the wall's
  // (measured 112 N before the wall was placed at all).
  void* wallscene = eng_scene_open(ctx, "mj_smoke_wall");
  check(wallscene != NULL, "a scene for the wall opens");
  check(eng_scene_add_box(wallscene, "no/such/mesh.stl", 0.0f, 0.0f, 0.0f) == -1 &&
            eng_scene_add_box(wallscene, NULL, 0.0f, 0.0f, 0.0f) == -1 &&
            eng_scene_add_box(NULL, wall_path, 0.0f, 0.0f, 0.0f) == -1,
        "a wall that cannot be loaded, and a null argument, are -1 and not a silent absence");
  char werr[256] = {0};
  void* wrobot = NULL;
  int wrc = eng_robot_attach_ex(wallscene, urdf, "link06", 1, 0.0, &wrobot, werr, sizeof(werr));
  printf("     attach_ex(weld_root=1) rc=%d err='%s'\n", wrc, werr);
  check(wrc == 0 && wrobot != NULL, "attach_ex with a welded root attaches");
  int wnv = eng_robot_num_dofs(wrobot);
  printf("     weld_root 1: nv=%d\n", wnv);
  check(wnv == 6, "weld_root 1 is the welded base");
  // the welded arm's own zero pose: the wall below is placed where its links are
  float wq[16] = {0};
  float wv[16] = {0};
  eng_robot_set_state(wrobot, wq, wv);
  stepn(wrobot, 200, 1.0e-4);
  float bare_fz = 0.0f;
  float bare_mag = 0.0f;
  contact_totals(wrobot, &bare_fz, &bare_mag);
  printf("     no wall: |f|=%.3f\n", bare_mag);
  check(bare_mag < 1.0f, "an arm with no ground and no wall touches nothing");
  check(eng_scene_add_box(wallscene, wall_path, 0.10f, 0.0f, 0.16f) == 0, "the wall is placed");
  stepn(wrobot, 200, 1.0e-4);
  float wall_fz = 0.0f;
  float wall_mag = 0.0f;
  contact_totals(wrobot, &wall_fz, &wall_mag);
  printf("     wall at (0.10,0,0.16): |f|=%.3f fz=%.3f\n", wall_mag, wall_fz);
  check(wall_mag > 1.0f, "and it is in the model: the arm presses on it");
  check(eng_robot_num_dofs(wrobot) == wnv,
        "the recompile the wall forces does not cost the robot its handle");
  // "once per scene, later calls are no-ops": the second call is made at a place the arm cannot reach, so
  // if it were applied the wall would leave and the contact above would read nothing at all
  check(eng_scene_add_box(wallscene, wall_path, 5.0f, 0.0f, 0.16f) == 0, "a second wall is accepted");
  stepn(wrobot, 200, 1.0e-4);
  float kept_mag = 0.0f;
  contact_totals(wrobot, &wall_fz, &kept_mag);
  printf("     after a second call the contact is |f|=%.3f\n", kept_mag);
  check(kept_mag > 1.0f, "as a no-op: the first wall did not move");

  eng_robot_free(robot);
  eng_scene_free(ctx, scene);
  eng_robot_free(frobot);
  eng_scene_free(ctx, scene2);
  eng_robot_free(exrobot);
  eng_scene_free(ctx, exscene);
  eng_robot_free(wrobot);
  eng_scene_free(ctx, wallscene);
  eng_context_close(ctx);

  printf("\n%s (%d failure%s)\n", failures == 0 ? "smoke passed" : "smoke FAILED", failures,
         failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
