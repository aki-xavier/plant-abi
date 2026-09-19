// mj_shim_smoke.c — drives the eng_* ABI the way the V control core does, so the MuJoCo backend can be
// checked without involving V: context, scene, ground, attach, state, step, per-link contact readout.
// It also pins that a part which is not ported fails with a message rather than a plausible zero.
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

int main(int argc, char** argv) {
  setvbuf(stdout, NULL, _IONBF, 0);  // a crash must not eat the progress report
  const char* root = (argc > 1) ? argv[1] : ".";
  static char urdf[1024];
  snprintf(urdf, sizeof(urdf), "%s/z1/z1.urdf", root);

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
  // rotvec, joints] and the DOF count grows by six (the arm's model stands in for the duck and the G1)
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

  eng_robot_free(robot);
  eng_scene_free(ctx, scene);
  eng_robot_free(frobot);
  eng_scene_free(ctx, scene2);
  eng_context_close(ctx);

  printf("\n%s (%d failure%s)\n", failures == 0 ? "smoke passed" : "smoke FAILED", failures,
         failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
