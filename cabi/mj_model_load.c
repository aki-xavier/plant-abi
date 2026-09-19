// mj_model_load.c — loads every model it is pointed at through the shim and checks the DOF count the
// model itself declares (an expectation from the models, not from this backend), which is the mesh
// path's test: MuJoCo reads binary STL only, so an ASCII mesh in a model is where this breaks.
//
// The models are ../control-model's, not this crate's, so the models DIRECTORY is an argument
// (default "."): this crate holds the ABI, and a self-check that hardcoded a neighbour's relative
// path broke silently the day those files moved out of the repository it used to be run from.
//
// Build: cabi/build_mj.sh, then cc -O2 -Icabi -o $BUILD/mj_model_load cabi/mj_model_load.c
//        -L$BUILD/bin -leng_shim -Wl,-rpath,$BUILD/bin -Wl,-rpath,$MJ
// Run:   mj_model_load <models-dir>

#include <stdio.h>
#include <string.h>

#include "eng_shim.h"

static int failures = 0;

static void check(int cond, const char* what, int got, int want) {
  printf("%s %-46s got=%d want=%d\n", cond ? "ok  " : "FAIL", what, got, want);
  if (!cond) {
    failures++;
  }
}

static int probe(void* ctx, const char* urdf, const char* ee, int floating, char* err, int cap,
                 char* note, size_t note_cap) {
  void* scene = eng_scene_open(ctx, urdf);
  if (scene == NULL) {
    snprintf(note, note_cap, "scene open failed");
    return -1;
  }
  void* robot = NULL;
  int rc = floating ? eng_robot_attach_floating(scene, urdf, ee, &robot, err, cap)
                    : eng_robot_attach(scene, urdf, ee, &robot, err, cap);
  int nv = -1;
  if (rc == 0 && robot != NULL) {
    nv = eng_robot_num_dofs(robot);
    static float q[64];
    static float v[64];
    memset(q, 0, sizeof(q));
    memset(v, 0, sizeof(v));
    if (eng_robot_set_state(robot, q, v) != 0) {
      snprintf(note, note_cap, "set_state failed");
      nv = -1;
    }
  } else {
    snprintf(note, note_cap, "%s", err);
  }
  eng_robot_free(robot);
  eng_scene_free(ctx, scene);
  return nv;
}

int main(int argc, char** argv) {
  setvbuf(stdout, NULL, _IONBF, 0);
  const char* root = (argc > 1) ? argv[1] : ".";
  void* ctx = eng_context_open();
  if (ctx == NULL) {
    printf("FAIL context open\n");
    return 1;
  }
  char err[512];
  char note[512];
  char z1_urdf[1024];
  char g1_urdf[1024];
  snprintf(z1_urdf, sizeof(z1_urdf), "%s/z1/z1.urdf", root);
  snprintf(g1_urdf, sizeof(g1_urdf), "%s/unitree_g1/unitree_g1.urdf", root);

  struct {
    const char* name;
    const char* urdf;
    const char* ee;
    int floating;
    int want;  // the DOF count this model declares
  } models[] = {
      {"z1 arm (fixed)", z1_urdf, "link06", 0, 6},
      {"unitree G1 (floating)", g1_urdf, "left_ankle_roll_link", 1, 35},
  };

  for (size_t i = 0; i < sizeof(models) / sizeof(models[0]); ++i) {
    err[0] = 0;
    note[0] = 0;
    int nv = probe(ctx, models[i].urdf, models[i].ee, models[i].floating, err, sizeof(err), note,
                   sizeof(note));
    if (nv < 0) {
      printf("FAIL %-24s could not load: %s\n", models[i].name, note[0] ? note : "unknown");
      failures++;
      continue;
    }
    check(nv == models[i].want, models[i].name, nv, models[i].want);
  }

  eng_context_close(ctx);
  printf("\n%s (%d failure%s)\n", failures == 0 ? "models loaded" : "model load FAILED", failures,
         failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
