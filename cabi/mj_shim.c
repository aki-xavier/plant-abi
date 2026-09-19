// mj_shim.c — the eng_* C ABI implemented on MuJoCo's C API; the V side binds these names and never
// learns which engine is behind them (layering rule: "the C world's only entry is eng_shim.h's C ABI
// face"). A scene IS its single robot's spec plus the scene's static geoms: the spec comes from the first
// attach, earlier statics are applied then, later ones are spliced in and the model recompiled.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>      // getcwd, for the absolute meshdir a floating model needs
#include <sys/stat.h>    // mkdir, for the mesh cache
#include <errno.h>       // EEXIST, when a cache directory is already there
#define GL_SILENCE_DEPRECATION 1  // CGL is how the offscreen context is made, and it is deprecated
#include <OpenGL/OpenGL.h>  // CGL, for the offscreen context MuJoCo's renderer draws into

#include <mujoco/mujoco.h>

// the contract this file implements: including it checks every definition below against the V side's bindings
#include "eng_shim.h"

// mj_debug_enabled answers SD_MJ_DEBUG once per process (its reads sit in per-contact loops and getenv
// walks the environment); MJ_CONTACT_REPORT/MJ_CONTACT_STIFFNESS are set in-process and stay uncached.
static int mj_debug_enabled(void) {
  static int cached = -1;
  if (cached < 0) {
    cached = getenv("SD_MJ_DEBUG") != NULL ? 1 : 0;
  }
  return cached;
}


#define MJ_MAX_ROBOTS 8
#define MJ_MAX_STATICS 8
#define MJ_MAX_CHILDREN 64  // bodies that may hang off one parent
#define MJ_ERR_CAP 512

// ---------------------------------------------------------------- handles --

typedef struct MjRobot MjRobot;
typedef struct MjScene MjScene;

// Forward declarations: the mesh preparation and the contact readout's model, both defined further down.
static long mj_count_facets(const char* path);
static int mj_copy_file(const char* src, const char* dst);
static int mj_write_decimated_stl(const char* src, const char* dst, long facets);
static int mj_mkdirp(const char* path);
static int mj_prepare_one_mesh(const char* src, char* dst, size_t cap);
static int mj_penalty_report(MjRobot* r);
static mjtNum mj_contact_stiffness(MjRobot* r);

// A scene's statics, remembered until (or applied after) the spec exists.
typedef struct MjStatic {
  int kind;               // 0 = ground plane, 1 = mesh-bearing box
  float a, b, c, d;       // plane: normal and distance
  char path[MJ_ERR_CAP];  // box: the mesh file
} MjStatic;

struct MjScene {
  mjSpec* spec;  // created by the first attach
  mjModel* model;
  mjData* data;
  MjRobot* robot;
  MjStatic statics[MJ_MAX_STATICS];
  int n_statics;
  int has_box;  // the contract makes eng_scene_add_box a once-per-scene call
  // MuJoCo's own renderer, built on first use: the scene it draws, the camera, the option set, and the
  // offscreen GL context the pixels come out of.
  mjvScene vis;
  mjvCamera cam;
  mjvOption opt;
  mjrContext con;
  int vis_ready;
  unsigned char* rgb;  // rgb_w * rgb_h * 3, reused across calls
  int rgb_w;
  int rgb_h;
  char render_error[MJ_ERR_CAP];
};

struct MjRobot {
  MjScene* scene;
  int n_joints;
  int nq;
  int nv;
  int* qpos_adr;  // [n_joints] into d->qpos
  int* dof_adr;   // [n_joints] into d->qvel / qfrc_applied
  // slot_of_body[node] is the model body id of V-side tree node `node` (0 = root); mj_resolve has the two orders.
  int* slot_of_body;
  int n_links;
  int ee_body;  // model body id of the end-effector link, -1 if unknown
  // the floating base: MuJoCo's free joint is a quaternion (7 slots) with a body-frame omega, this ABI a rotation vector (3) and a world-frame omega (base_dofs 6 floating, 0 welded).
  int base_dofs;
  int base_qpos_adr;
  int base_dof_adr;
  // The contact REPORT's mode and stiffness: stated by eng_robot_set_contact_report, otherwise asked of the environment (tests/g1_attach.rs relies on it).
  int report_set;
  int report_penalty;
  double report_stiffness;
  // The contact the robot's own commanded pose creates, captured per body BEFORE a step (a solver constraint resolves its overlap within the step).
  double* pen;    // [nbody]
  double* pen_n;  // [nbody * 3]
  double* pen_p;  // [nbody * 3]
  // pen_w/pen_wsum are the penetration-weighted sum of the body's own contact points: the patch centroid
  // the penalty readout applies its normal force at (the deepest point is a sole corner and jumps).
  double* pen_w;     // [nbody * 3]
  double* pen_wsum;  // [nbody]
  // The reported force is low-passed (a rigid contact's first tick is an impulse); a held load passes.
  float* fc_filt;   // [n_links]
  float* fw_filt;   // [n_links * 6]
  double last_dt;   // simulated time the last step advanced, for the filter
  char last_error[MJ_ERR_CAP];
};

typedef struct MjContext {
  mjVFS vfs;
  int vfs_ready;
} MjContext;

// mj_set_err formats a failure into the caller's buffer (the capacity is the CALLER's, so a long message
// is truncated rather than overrunning it) and mirrors it to stderr, as eng_shim.h promises. A capless
// call (cap 0) still reaches stderr.
static void mj_set_err(char* dst, size_t cap, const char* msg) {
  fprintf(stderr, "%s\n", msg);
  if (dst == NULL || cap == 0) {
    return;
  }
  snprintf(dst, cap, "%s", msg);
}

// ------------------------------------------------------------- context -----

void* eng_context_open(void) {
  MjContext* c = (MjContext*)calloc(1, sizeof(MjContext));
  if (c == NULL) {
    return NULL;
  }
  mj_defaultVFS(&c->vfs);
  c->vfs_ready = 1;
  return c;
}

void eng_context_close(void* ctx) {
  if (ctx == NULL) {
    return;
  }
  MjContext* c = (MjContext*)ctx;
  if (c->vfs_ready) {
    mj_deleteVFS(&c->vfs);
  }
  free(c);
}

// --------------------------------------------------------------- scene -----

// eng_scene_open starts an empty scene: the spec is created by the first attach, so a scene with no robot
// yet has nothing to compile.
void* eng_scene_open(void* ctx, const char* name) {
  if (ctx == NULL) {
    return NULL;
  }
  MjScene* s = (MjScene*)calloc(1, sizeof(MjScene));
  if (s == NULL) {
    return NULL;
  }
  (void)name;
  return s;
}

void eng_scene_free(void* ctx, void* scene) {
  (void)ctx;
  if (scene == NULL) {
    return;
  }
  MjScene* s = (MjScene*)scene;
  MjRobot* r = s->robot;
  if (r != NULL) {
    free(r->qpos_adr);
    free(r->dof_adr);
    free(r->slot_of_body);
    free(r->pen);
    free(r->pen_n);
    free(r->pen_p);
    free(r->pen_w);
    free(r->pen_wsum);
    free(r->fc_filt);
    free(r->fw_filt);
    free(r);
  }
  if (s->data != NULL) {
    mj_deleteData(s->data);
  }
  if (s->vis_ready != 0) {
    // the visualisation goes before the context that drew it, the order the engine's own viewer frees them in
    mjv_freeScene(&s->vis);
    mjr_freeContext(&s->con);
  }
  free(s->rgb);
  if (s->model != NULL) {
    mj_deleteModel(s->model);
  }
  if (s->spec != NULL) {
    mj_deleteSpec(s->spec);
  }
  free(s);
}

// apply_static adds one recorded static to a spec: the ground plane, or a mesh and a geom referencing it.
static int apply_static(mjSpec* spec, const MjStatic* st) {
  mjsBody* world = mjs_findBody(spec, "world");
  if (world == NULL) {
    return -1;
  }
  if (st->kind == 0) {
    mjsGeom* g = mjs_addGeom(world, NULL);
    if (g == NULL) {
      return -1;
    }
    g->type = mjGEOM_PLANE;
    // The plane's COLLISION is the analytic half-space: size only sets the quad DRAWN and the model's
    // extent (a 1e6 quad puts every camera inside the near plane and the frame comes back black).
    g->size[0] = 10.0;
    g->size[1] = 10.0;
    g->size[2] = 1.0;  // MuJoCo rejects a plane whose third size is not positive
    // The contract is the plane n . x = distance; a MuJoCo plane's surface is the +z half-space of the
    // geom frame, so a non-+z normal is carried by the geom's rotation, the offset along it.
    mjtNum n[3] = {st->a, st->b, st->c};
    mjtNum len = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len > 1e-12) {
      n[0] /= len;
      n[1] /= len;
      n[2] /= len;
    } else {
      n[0] = 0.0;
      n[1] = 0.0;
      n[2] = 1.0;
    }
    mjtNum q[4];
    mju_quatZ2Vec(q, n);
    for (int i = 0; i < 4; ++i) {
      g->quat[i] = q[i];
    }
    for (int i = 0; i < 3; ++i) {
      g->pos[i] = (mjtNum)st->d * n[i];
    }
    return 0;
  }
  mjsMesh* mesh = mjs_addMesh(spec, NULL);
  if (mesh == NULL) {
    return -1;
  }
  // The geom references the mesh by NAME, so the mesh needs one (a file path compiles to "mesh not
  // found", which reaches the caller as a static that silently never appeared — an absent wall).
  mjs_setName(mesh->element, "mj_shim_static_mesh");
  mjs_setString(mesh->file, st->path);
  mjsGeom* g = mjs_addGeom(world, NULL);
  if (g == NULL) {
    return -1;
  }
  g->type = mjGEOM_MESH;
  mjs_setString(g->meshname, "mj_shim_static_mesh");
  g->pos[0] = st->a;
  g->pos[1] = st->b;
  g->pos[2] = st->c;
  return 0;
}

// Two collision classes: the robot's links against the scene's statics and nothing against their own kind
// (MuJoCo collides every pair of geoms by default, so a folded pose piles up contacts on a robot held in
// the air); they live on the COMPILED model, so every compile rebuilds them. The contact reference could
// not stay at MuJoCo's default either: its 20 ms time constant needs tens of milliseconds to build force,
// so an arm on a stiff position loop walks into a wall for that whole window. The replaced engine's
// contact was a distance-field penalty (5e7 Pa/m); gap opens a force-generating buffer outside the
// surface and tau/zeta match its force per penetration. All three are env-settable.
#define MJ_CONTACT_TAU 0.05
#define MJ_CONTACT_ZETA 1.0
#define MJ_CONTACT_GAP 0.002
// 50 ms is the softest contact the repository's tuning tolerates and the value every model reads at: the
// wall push holds a 15 N steady load with a 3 N first-tick step (stiffer through MJ_CONTACT_TAU).
static double mj_env_num(const char* name, double fallback) {
  const char* v = getenv(name);
  if (v == NULL || v[0] == 0) {
    return fallback;
  }
  return atof(v);
}
static void mj_apply_contact_policy(mjModel* m) {
  const double tau = mj_env_num("MJ_CONTACT_TAU", MJ_CONTACT_TAU);
  const double zeta = mj_env_num("MJ_CONTACT_ZETA", MJ_CONTACT_ZETA);
  const double gap = mj_env_num("MJ_CONTACT_GAP", MJ_CONTACT_GAP);
  for (int g = 0; g < m->ngeom; ++g) {
    if (m->geom_bodyid[g] == 0) {  // the world body carries the statics
      m->geom_contype[g] = 2;
      m->geom_conaffinity[g] = 1;
    } else {
      m->geom_contype[g] = 1;
      m->geom_conaffinity[g] = 2;
    }
    m->geom_solref[2 * g + 0] = tau;
    m->geom_solref[2 * g + 1] = zeta;
    m->geom_gap[g] = gap;
  }
}

// record_or_apply remembers a static when no robot is attached yet, else splices it into the live spec
// and recompiles so the caller's next steps see it; the contract allows either order.
static int record_or_apply(MjScene* s, const MjStatic* st) {
  if (s->spec == NULL) {
    if (s->n_statics >= MJ_MAX_STATICS) {
      return -1;
    }
    s->statics[s->n_statics++] = *st;
    return 0;
  }
  if (apply_static(s->spec, st) != 0) {
    return -1;
  }
  mjModel* m = mj_compile(s->spec, NULL);
  if (m == NULL) {
    // the caller gets only a status, so the compiler's own message is printed here: a failed recompile
    // otherwise leaves the previous model in place and every later read looks plausible
    fprintf(stderr, "mj_shim: recompiling the scene after a static failed: %s\n",
            mjs_getError(s->spec));
    return -1;
  }
  mj_apply_contact_policy(m);
  mjData* d = mj_makeData(m);
  if (mj_debug_enabled()) {
    fprintf(stderr, "mj_shim: scene recompiled: nbody=%d ngeom=%d\n", (int)m->nbody, (int)m->ngeom);
    for (int g = 0; g < m->ngeom; ++g) {
      int mid = (int)m->geom_dataid[g];
      fprintf(stderr, "mj_shim:   geom[%d] body=%-12s type=%d mesh_verts=%d mesh_faces=%d\n", g,
              mj_id2name(m, mjOBJ_BODY, (int)m->geom_bodyid[g]), (int)m->geom_type[g],
              mid >= 0 ? (int)m->mesh_vertnum[mid] : 0, mid >= 0 ? (int)m->mesh_facenum[mid] : 0);
    }
    // where every body sits: placing a static so it misses the robot needs these (a mesh link's hull
    // reaches beyond what its name suggests)
    for (int b = 1; b < (int)m->nbody; ++b) {
      fprintf(stderr, "mj_shim:   body[%d] %-14s pos=[%.4f %.4f %.4f]\n", b,
              mj_id2name(m, mjOBJ_BODY, b), d->xpos[3 * b + 0], d->xpos[3 * b + 1],
              d->xpos[3 * b + 2]);
    }
  }
  if (s->data != NULL) {
    mj_deleteData(s->data);
  }
  if (s->model != NULL) {
    mj_deleteModel(s->model);
  }
  s->model = m;
  s->data = d;
  return 0;
}

int eng_scene_add_plane(void* scene, float nx, float ny, float nz, float distance) {
  if (scene == NULL) {
    return -1;
  }
  MjStatic st;
  memset(&st, 0, sizeof(st));
  st.kind = 0;
  st.a = nx;
  st.b = ny;
  st.c = nz;
  st.d = distance;
  return record_or_apply((MjScene*)scene, &st);
}

// eng_scene_add_box places a mesh-bearing body: MuJoCo needs the mesh in the same spec, so the file is
// added as a mesh and a geom referencing it. Once per scene, so later calls are no-ops rather than errors.
int eng_scene_add_box(void* scene, const char* path, float px, float py, float pz) {
  if (scene == NULL || path == NULL) {
    return -1;
  }
  MjScene* s = (MjScene*)scene;
  if (s->has_box) {
    return 0;
  }
  MjStatic st;
  memset(&st, 0, sizeof(st));
  st.kind = 1;
  st.a = px;
  st.b = py;
  st.c = pz;
  if (mj_prepare_one_mesh(path, st.path, sizeof(st.path)) != 0) {
    return -1;
  }
  if (record_or_apply(s, &st) != 0) {
    return -1;
  }
  s->has_box = 1;
  return 0;
}

// --------------------------------------------------------- compile pass ----

// mj_name_cmp sorts two bodies by name, which is the whole of the ordering rule
// below.
static int mj_name_cmp(mjModel* m, int a, int b) {
  if (a == b) {
    return 0;
  }
  const char* na = mj_id2name(m, mjOBJ_BODY, a);
  const char* nb = mj_id2name(m, mjOBJ_BODY, b);
  if (na == NULL) {
    return -1;
  }
  if (nb == NULL) {
    return 1;
  }
  int c = strcmp(na, nb);
  return c < 0 ? -1 : (c > 0 ? 1 : 0);
}

// mj_resolve builds the slot mapping between this ABI and MuJoCo's own order: the V-side tree (and so its
// q, tau and contact-slot indices) is a DFS pre-order with each body's children sorted by link name, while
// MuJoCo's bodies follow the model document's order, so every index that crosses is translated here. The
// node list is not per body: the root takes one node and then ONE NODE PER JOINT (the z1's six joints sit
// on five bodies), which makes the mapping independent of the engine's layout. slot_of_body[node] is the
// body id behind node `node` (0 = root); qpos_adr/dof_adr hold the joints in V-side order.
static int mj_resolve(MjScene* s, MjRobot* r, const char* root_link) {
  mjModel* m = s->model;
  // the robot's root: the body directly under the world
  int root = -1;
  for (int b = 1; b < m->nbody; ++b) {
    if (m->body_parentid[b] == 0) {
      root = b;
      break;
    }
  }
  if (root < 0) {
    return -1;
  }
  // DFS pre-order, children sorted by name
  int* order = (int*)calloc((size_t)m->nbody, sizeof(int));
  int* stack = (int*)calloc((size_t)m->nbody, sizeof(int));
  if (order == NULL || stack == NULL) {
    free(order);
    free(stack);
    return -1;
  }
  int n_order = 0;
  int sp = 0;
  stack[sp++] = root;
  while (sp > 0) {
    int cur = stack[--sp];
    if (n_order < m->nbody) {
      order[n_order++] = cur;
    }
    int kids[MJ_MAX_CHILDREN];
    int nk = 0;
    for (int b = 1; b < m->nbody && nk < MJ_MAX_CHILDREN; ++b) {
      if (m->body_parentid[b] == cur) {
        kids[nk++] = b;
      }
    }
    for (int i = 0; i < nk; ++i) {
      for (int j = i + 1; j < nk; ++j) {
        if (mj_name_cmp(m, kids[j], kids[i]) < 0) {
          int t = kids[i];
          kids[i] = kids[j];
          kids[j] = t;
        }
      }
    }
    for (int i = nk - 1; i >= 0; --i) {
      stack[sp++] = kids[i];
    }
  }
  free(stack);
  if (n_order != m->nbody - 1) {
    free(order);
    return -1;  // the robot's bodies are not one tree under its root
  }

  // node 0 is the root, then one node per joint in the same canonical order
  int node = 1;
  for (int i = 0; i < n_order && node <= r->n_joints; ++i) {
    int b = order[i];
    for (int j = 0; j < (int)m->body_jntnum[b] && node <= r->n_joints; ++j) {
      int jid = (int)m->body_jntadr[b] + j;
      if (m->jnt_type[jid] == mjJNT_FREE) {
        continue;  // the base's own joint is not a V-side node
      }
      r->qpos_adr[node - 1] = (int)m->jnt_qposadr[jid];
      r->dof_adr[node - 1] = (int)m->jnt_dofadr[jid];
      r->slot_of_body[node] = b;
      node++;
    }
  }
  const int n_joints = node - 1;
  free(order);
  if (n_joints <= 0) {
    return -1;  // nothing to drive
  }
  // the root's own node: a floating base has a body for it, while a fixed base's welded root link is
  // folded into the world body, whose geoms are the scene's statics and not this robot's.
  int root_body = 0;
  if (root_link != NULL && root_link[0] != 0) {
    int found = mj_name2id(m, mjOBJ_BODY, root_link);
    if (found > 0) {
      root_body = found;
    }
  } else if (r->base_dofs == 6) {
    root_body = root;
  }
  r->slot_of_body[0] = root_body;
  r->n_joints = n_joints;
  r->n_links = n_joints + 1;
  return 0;
}

// attach_impl is the shared attach body: the scene's spec IS the robot's spec, so this parses the URDF
// into it, applies whatever statics arrived before it, compiles, and resolves the robot's joint slots
// (the joints keep their model names, so nothing is prefixed here).
// mj_find_root_link writes the URDF's root link into out: the link no joint declares as a child, since
// the file's first <link> would be a guess and a wrong root attaches the floating joint to a wrong body.
static int mj_find_root_link(const char* txt, char* out, size_t cap) {
  const char* p = txt;
  while ((p = strstr(p, "<link name=\"")) != NULL) {
    const char* q = p + strlen("<link name=\"");
    const char* e = strchr(q, '"');
    if (e == NULL) {
      break;
    }
    size_t len = (size_t)(e - q);
    if (len + 1 < cap) {
      snprintf(out, cap, "%.*s", (int)len, q);
      char pat[512];
      snprintf(pat, sizeof(pat), "<child link=\"%s\"", out);
      if (strstr(txt, pat) == NULL) {
        return 0;
      }
    }
    p = e;
  }
  return -1;
}

// ---- mesh preparation -------------------------------------------------------
// MuJoCo's mesh loader reads binary STL only while these models ship a mix (about half of the G1's meshes
// are ASCII, which it refuses), so nothing in the tree is rewritten: every referenced mesh is converted or
// copied into a cache directory and the model pointed at that. The format test must not look at the
// 80-byte header, which is free text and does hold "solid": file_size == 84 + 50 * facets, count at 80.

#define MJ_MESH_CACHE_DEFAULT "/tmp/mj_mesh_cache"

static void mj_dir_of(const char* path, char* out, size_t cap) {
  out[0] = 0;
  const char* slash = strrchr(path, '/');
  if (slash != NULL) {
    size_t len = (size_t)(slash - path);
    if (len < cap) {
      snprintf(out, cap, "%.*s", (int)len, path);
    }
  }
  if (out[0] != '/') {
    char cwd[768] = {0};
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
      char abs[1024] = {0};
      if (out[0] == 0) {
        snprintf(abs, sizeof(abs), "%s", cwd);
      } else {
        snprintf(abs, sizeof(abs), "%s/%s", cwd, out);
      }
      snprintf(out, cap, "%s", abs);
    }
  }
}

static int mj_mkdirp(const char* path) {
  char tmp[1024];
  snprintf(tmp, sizeof(tmp), "%s", path);
  for (char* p = tmp + 1; *p != 0; ++p) {
    if (*p == '/') {
      *p = 0;
      mkdir(tmp, 0777);
      *p = '/';
    }
  }
  return mkdir(tmp, 0777) == 0 || errno == EEXIST ? 0 : -1;
}

// MuJoCo's STL decoder refuses a mesh with more than 200000 facets (a real hand's palm link has 287456).
// A mesh geom collides as the convex hull of its vertices, so keeping every k-th facet brings the count
// under the cap and leaves that hull identical up to the surface a dropped facet spans (under a mm here).
#define MJ_STL_FACE_CAP 200000

// The cache keys on the reference path, so the preparation version is part of the cache path: bumping it
// retires every cached file and a stale one cannot be mistaken for a current one.
#define MJ_MESH_PREP_VERSION 2
static long mj_count_facets(const char* path) {
  FILE* f = fopen(path, "rb");
  if (f == NULL) {
    return -1;
  }
  unsigned char head[84];
  size_t got = fread(head, 1, sizeof(head), f);
  long size = -1;
  if (fseek(f, 0, SEEK_END) == 0) size = ftell(f);
  fclose(f);
  if (got < sizeof(head) || size < 0) {
    return -1;
  }
  unsigned int n = (unsigned int)head[80] | ((unsigned int)head[81] << 8) |
                   ((unsigned int)head[82] << 16) | ((unsigned int)head[83] << 24);
  if (size != 84L + 50L * (long)n) {
    return -1;
  }
  return (long)n;
}

// mj_write_decimated_stl keeps every k-th facet of a mesh over the decoder's cap (k the smallest step
// that brings it under), still a valid mesh of real triangles whose hull lies inside the original one.
static int mj_write_decimated_stl(const char* src, const char* dst, long facets) {
  long step = (facets + MJ_STL_FACE_CAP - 1) / MJ_STL_FACE_CAP;
  if (step < 1) {
    step = 1;
  }
  long kept = (facets + step - 1) / step;
  if (kept > MJ_STL_FACE_CAP) {
    return -1;
  }
  FILE* in = fopen(src, "rb");
  if (in == NULL) {
    return -1;
  }
  if (fseek(in, 84, SEEK_SET) != 0) {
    fclose(in);
    return -1;
  }
  FILE* out = fopen(dst, "wb");
  if (out == NULL) {
    fclose(in);
    return -1;
  }
  unsigned char hdr[80] = {0};
  snprintf((char*)hdr, sizeof(hdr), "mj_shim: every %ldth facet, %ld of %ld", step, kept, facets);
  unsigned int n32 = (unsigned int)kept;
  unsigned char cnt[4] = {(unsigned char)(n32 & 0xff), (unsigned char)((n32 >> 8) & 0xff),
                          (unsigned char)((n32 >> 16) & 0xff), (unsigned char)((n32 >> 24) & 0xff)};
  if (fwrite(hdr, 1, sizeof(hdr), out) != sizeof(hdr) || fwrite(cnt, 1, sizeof(cnt), out) != 4) {
    fclose(in);
    fclose(out);
    return -1;
  }
  long written = 0;
  int rc = 0;
  for (long f = 0; f < facets; ++f) {
    float rec[12];
    unsigned char attr[2];
    if (fread(rec, sizeof(float), 12, in) != 12 || fread(attr, 1, 2, in) != 2) {
      rc = -1;
      break;
    }
    if (f % step != 0) {
      continue;
    }
    if (fwrite(rec, sizeof(float), 12, out) != 12 || fwrite(attr, 1, 2, out) != 2) {
      rc = -1;
      break;
    }
    written++;
  }
  fclose(in);
  if (fclose(out) != 0) {
    rc = -1;
  }
  if (rc != 0 || written != kept) {
    return -1;
  }
  fprintf(stderr, "mj_shim: %s had %ld facets, past MuJoCo's %d cap: kept every %ldth, %ld facets\n",
          src, facets, MJ_STL_FACE_CAP, step, kept);
  return 0;
}

// mj_count_facets answers both questions: a negative answer means the file is not a binary STL of that
// shape (so it is ASCII and must be converted), a non-negative one is the facet count the decoder sees.
static long mj_count_facets(const char* path);

static int mj_ascii_token(const char** p, const char* end, char* out, size_t cap) {
  const char* q = *p;
  while (q < end && (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r')) {
    q++;
  }
  if (q >= end) {
    *p = q;
    return 0;
  }
  size_t n = 0;
  while (q < end && *q != ' ' && *q != '\t' && *q != '\n' && *q != '\r') {
    if (n + 1 < cap) {
      out[n++] = *q;
    }
    q++;
  }
  out[n] = 0;
  *p = q;
  return 1;
}

// mj_ascii_float reads one number from the token stream, or leaves the caller's value alone and answers 0.
static int mj_ascii_float(const char** p, const char* end, float* out) {
  char tok[64];
  if (mj_ascii_token(p, end, tok, sizeof(tok)) == 0) {
    return 0;
  }
  return sscanf(tok, "%f", out) == 1 ? 1 : 0;
}

// mj_convert_ascii_stl rewrites one ASCII STL as binary, walking the file's whitespace-separated TOKENS
// rather than its lines: an STL need not put one directive per line (the wall mesh here carries a facet's
// "endfacet" and the next "facet normal" on one line), and a line-oriented parse loses every facet after
// the first. A converted file whose record count does not match its facet count is refused and removed.
static int mj_convert_ascii_stl(const char* src, const char* dst) {
  FILE* in = fopen(src, "rb");
  if (in == NULL) {
    return -1;
  }
  if (fseek(in, 0, SEEK_END) != 0) {
    fclose(in);
    return -1;
  }
  long size = ftell(in);
  if (size <= 0 || fseek(in, 0, SEEK_SET) != 0) {
    fclose(in);
    return -1;
  }
  char* buf = (char*)malloc((size_t)size + 1);
  if (buf == NULL) {
    fclose(in);
    return -1;
  }
  size_t got = fread(buf, 1, (size_t)size, in);
  fclose(in);
  buf[got] = 0;
  const char* end = buf + got;

  long facets = 0;
  const char* q = buf;
  char tok[64];
  while (mj_ascii_token(&q, end, tok, sizeof(tok)) != 0) {
    if (strcmp(tok, "facet") == 0 && mj_ascii_token(&q, end, tok, sizeof(tok)) != 0 &&
        strcmp(tok, "normal") == 0) {
      facets++;
    }
  }
  if (facets <= 0) {
    free(buf);
    return -1;
  }

  FILE* out = fopen(dst, "wb");
  if (out == NULL) {
    free(buf);
    return -1;
  }
  unsigned char hdr[80] = {0};
  snprintf((char*)hdr, sizeof(hdr), "mj_shim: converted from ASCII STL");
  unsigned int n32 = (unsigned int)facets;
  unsigned char cnt[4] = {(unsigned char)(n32 & 0xff), (unsigned char)((n32 >> 8) & 0xff),
                          (unsigned char)((n32 >> 16) & 0xff), (unsigned char)((n32 >> 24) & 0xff)};
  int rc = 0;
  if (fwrite(hdr, 1, sizeof(hdr), out) != sizeof(hdr) || fwrite(cnt, 1, sizeof(cnt), out) != 4) {
    rc = -1;
  }
  float rec[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int vi = 0;
  long written = 0;
  q = buf;
  while (rc == 0 && mj_ascii_token(&q, end, tok, sizeof(tok)) != 0) {
    if (strcmp(tok, "facet") == 0) {
      // a facet begins: the previous one is complete, so write it, then read this one's normal
      if (vi == 3) {
        unsigned char attr[2] = {0, 0};
        if (fwrite(rec, sizeof(float), 12, out) != 12 ||
            fwrite(attr, 1, sizeof(attr), out) != 2) {
          rc = -1;
          break;
        }
        written++;
      }
      vi = 0;
      rec[0] = rec[1] = rec[2] = 0.0f;
      if (mj_ascii_token(&q, end, tok, sizeof(tok)) == 0) {  // the "normal" keyword
        break;
      }
      for (int k = 0; k < 3; ++k) {
        float v = 0.0f;
        mj_ascii_float(&q, end, &v);
        rec[k] = v;
      }
    } else if (strcmp(tok, "vertex") == 0) {
      float x = 0.0f, y = 0.0f, z = 0.0f;
      mj_ascii_float(&q, end, &x);
      mj_ascii_float(&q, end, &y);
      mj_ascii_float(&q, end, &z);
      if (vi < 3) {
        rec[3 + vi * 3 + 0] = x;
        rec[3 + vi * 3 + 1] = y;
        rec[3 + vi * 3 + 2] = z;
        vi++;
      }
    }
  }
  if (rc == 0 && vi == 3) {
    unsigned char attr[2] = {0, 0};
    if (fwrite(rec, sizeof(float), 12, out) != 12 || fwrite(attr, 1, sizeof(attr), out) != 2) {
      rc = -1;
    } else {
      written++;
    }
  }
  if (fclose(out) != 0) {
    rc = -1;
  }
  free(buf);
  if (rc != 0 || written != facets) {
    remove(dst);
    return -1;
  }
  return 0;
}

static int mj_copy_file(const char* src, const char* dst) {
  FILE* in = fopen(src, "rb");
  if (in == NULL) {
    return -1;
  }
  FILE* out = fopen(dst, "wb");
  if (out == NULL) {
    fclose(in);
    return -1;
  }
  char buf[1 << 16];
  size_t got;
  while ((got = fread(buf, 1, sizeof(buf), in)) > 0) {
    if (fwrite(buf, 1, got, out) != got) {
      fclose(in);
      fclose(out);
      return -1;
    }
  }
  fclose(in);
  fclose(out);
  return 0;
}

// mj_prepare_one_mesh prepares a single mesh file into the cache and returns the path to use in its place:
// a scene's static mesh arrives as an absolute path and bypasses the model's own mesh scan.
static int mj_prepare_one_mesh(const char* src, char* dst, size_t cap) {
  const char* cache = getenv("MJ_MESH_CACHE_DIR");
  if (cache == NULL || cache[0] == 0) {
    cache = MJ_MESH_CACHE_DEFAULT;
  }
  // the key is the source path, so two scenes asking for the same file share the prepared copy
  unsigned int h = 2166136261u;
  for (const char* q = src; *q != 0; ++q) {
    h = (h ^ (unsigned char)*q) * 16777619u;
  }
  snprintf(dst, cap, "%s/v%d/static_%08x.stl", cache, MJ_MESH_PREP_VERSION, h);
  if (mj_mkdirp(cache) != 0) {
    return -1;
  }
  char subdir[1600];
  snprintf(subdir, sizeof(subdir), "%s/v%d", cache, MJ_MESH_PREP_VERSION);
  if (mj_mkdirp(subdir) != 0) {
    return -1;
  }
  FILE* have = fopen(dst, "rb");
  if (have != NULL) {
    long dsize = 0;
    if (fseek(have, 0, SEEK_END) == 0) {
      dsize = ftell(have);
    }
    fclose(have);
    if (dsize > 0) {
      return 0;  // already prepared
    }
  }
  long facets = mj_count_facets(src);
  int rc;
  if (facets > MJ_STL_FACE_CAP) {
    rc = mj_write_decimated_stl(src, dst, facets);
  } else if (facets >= 0) {
    rc = mj_copy_file(src, dst);
  } else {
    rc = mj_convert_ascii_stl(src, dst);
  }
  if (rc != 0) {
    remove(dst);
    fprintf(stderr, "mj_shim: cannot prepare the static's mesh %s\n", src);
    return -1;
  }
  if (mj_debug_enabled()) {
    fprintf(stderr, "mj_shim: static mesh %s -> %s\n", src, dst);
  }
  return 0;
}

// mj_drop_unusable_blocks removes <collision>/<visual> blocks whose geometry MuJoCo cannot use (the duck's
// URDF references meshes committed empty, the G1's has collision elements with no <geometry>) and reports
// each one: a dropped block loses that geometry, never gains a wrong one, and the loss is never silent.
static int mj_drop_unusable_blocks(const char* text, const char* model_dir, char** out) {
  size_t cap = strlen(text) + 1;
  char* buf = (char*)malloc(cap);
  if (buf == NULL) {
    return -1;
  }
  size_t at = 0;
  const char* p = text;
  int dropped = 0;
  while (*p != 0) {
    const char* open = NULL;
    int is_collision = 1;
    const char* c1 = strstr(p, "<collision");
    const char* v1 = strstr(p, "<visual");
    if (c1 != NULL && (v1 == NULL || c1 < v1)) {
      open = c1;
    } else if (v1 != NULL) {
      open = v1;
      is_collision = 0;
    }
    if (open == NULL) {
      break;
    }
    const char* close_tag = is_collision ? "</collision>" : "</visual>";
    const char* close = strstr(open, close_tag);
    if (close == NULL) {
      break;
    }
    size_t block_len = (size_t)(close - open) + strlen(close_tag);
    int usable = strstr(open, "<geometry") != NULL && (size_t)(strstr(open, "<geometry") - open) < block_len;
    if (usable) {
      const char* mesh = strstr(open, "filename=\"");
      if (mesh != NULL && (size_t)(mesh - open) < block_len) {
        const char* s = mesh + strlen("filename=\"");
        const char* e = strchr(s, '"');
        if (e != NULL && (size_t)(e - open) < block_len) {
          char ref[512];
          snprintf(ref, sizeof(ref), "%.*s", (int)(e - s), s);
          char path[1600];
          snprintf(path, sizeof(path), "%s/%s", model_dir, ref);
          struct stat st;
          if (stat(path, &st) != 0 || st.st_size == 0) {
            usable = 0;  // an absent or empty mesh: the link keeps no geometry
          }
        }
      }
    }
    memcpy(buf + at, p, (size_t)(open - p));
    at += (size_t)(open - p);
    if (usable) {
      memcpy(buf + at, open, block_len);
      at += block_len;
    } else {
      dropped++;
    }
    p = close + strlen(close_tag);
  }
  memcpy(buf + at, p, strlen(p) + 1);
  at += strlen(p);
  if (dropped > 0) {
    fprintf(stderr,
            "mj_shim: dropped %d geometry block(s) MuJoCo cannot use (no <geometry>, or an empty\n"
            "mj_shim: or missing mesh file) — those links carry no such collision in this engine\n",
            dropped);
  }
  *out = buf;
  return 0;
}

// mj_prepare_meshes puts every mesh the model references into the cache in the model's own layout, and returns the meshdir to point at.
static int mj_prepare_meshes(const char* model_path, const char* text, char* meshdir, size_t cap,
                             char* err, int err_cap) {
  char srcdir[1024];
  mj_dir_of(model_path, srcdir, sizeof(srcdir));
  const char* cache = getenv("MJ_MESH_CACHE_DIR");
  if (cache == NULL || cache[0] == 0) {
    cache = MJ_MESH_CACHE_DEFAULT;
  }
  if (mj_mkdirp(cache) != 0) {
    snprintf(err, (size_t)err_cap, "mj_shim: cannot create the mesh cache at %s", cache);
    return -1;
  }
  snprintf(meshdir, cap, "%s/v%d", cache, MJ_MESH_PREP_VERSION);

  const char* markers[2] = {"filename=\"", "file=\""};
  for (int k = 0; k < 2; ++k) {
    const char* q = text;
    while ((q = strstr(q, markers[k])) != NULL) {
      if (k == 1 && q > text && q[-1] == 'e') {  // the "file" inside "filename"
        q += strlen(markers[k]);
        continue;
      }
      const char* s = q + strlen(markers[k]);
      const char* e = strchr(s, '"');
      if (e == NULL) {
        break;
      }
      char ref[512];
      snprintf(ref, sizeof(ref), "%.*s", (int)(e - s), s);
      q = e;
      if (ref[0] == 0 || ref[0] == '/') {
        continue;  // absolute references are the model's business, not the cache's
      }
      if (strstr(ref, "..") != NULL) {
        continue;  // nothing in these models, and the cache cannot mirror it
      }
      char src[1600];
      snprintf(src, sizeof(src), "%s/%s", srcdir, ref);
      char dst[1600];
      snprintf(dst, sizeof(dst), "%s/%s", meshdir, ref);
      FILE* probe = fopen(src, "rb");
      if (probe == NULL) {
        continue;  // not a mesh path we can act on (a texture, an absent asset)
      }
      long ssize = 0;
      if (fseek(probe, 0, SEEK_END) == 0) {
        ssize = ftell(probe);
      }
      fclose(probe);
      FILE* have = fopen(dst, "rb");
      if (have != NULL) {
        long dsize = 0;
        if (fseek(have, 0, SEEK_END) == 0) {
          dsize = ftell(have);
        }
        fclose(have);
        if (dsize == ssize || dsize > 0) {  // already prepared once
          continue;
        }
      }
      char subdir[1600];
      snprintf(subdir, sizeof(subdir), "%s", dst);
      char* lastslash = strrchr(subdir, '/');
      if (lastslash != NULL) {
        *lastslash = 0;
        if (mj_mkdirp(subdir) != 0) {
          snprintf(err, (size_t)err_cap, "mj_shim: cannot create the mesh cache subdirectory %s",
                   subdir);
          return -1;
        }
      }
      long facets = mj_count_facets(src);
      int rc;
      if (facets > MJ_STL_FACE_CAP) {
        rc = mj_write_decimated_stl(src, dst, facets);
      } else if (facets >= 0) {
        rc = mj_copy_file(src, dst);
      } else {
        rc = mj_convert_ascii_stl(src, dst);
      }
      if (rc != 0) {
        remove(dst);
        snprintf(err, (size_t)err_cap,
                 "mj_shim: could not prepare the mesh %s (is it an STL MuJoCo can read?)", ref);
        return -1;
      }
      if (mj_debug_enabled()) {
        fprintf(stderr, "mj_shim: mesh %s -> %s\n", ref, dst);
      }
    }
  }
  return 0;
}

// mj_float_urdf turns a URDF into the text of one whose root hangs off a world link by a floating joint
// (the URDF way to say "free base"), with two narrow edits: an absolute meshdir on the <compiler>, and the
// world link plus the floating joint before </robot>. Text work, because mjs_addFreeJoint segfaults on
// parsed bodies and saving a parsed spec as MJCF moves the root link's geometry into the worldbody.
// mj_model_text reads a model and returns the text to parse (meshes prepared, meshdir pointed at them,
// floating block appended); mj_inject_meshdir sets that meshdir, REPLACING a declared one.
static size_t mj_inject_meshdir(const char* txt, const char* dir, char* out, size_t cap) {
  const char* comp = strstr(txt, "<compiler");
  if (comp == NULL) {
    const char* robot = strstr(txt, "<robot");
    const char* tag_end = robot != NULL ? strchr(robot, '>') : NULL;
    if (tag_end == NULL) {
      return (size_t)snprintf(out, cap, "%s", txt);
    }
    size_t keep = (size_t)(tag_end - txt) + 1;
    memcpy(out, txt, keep);
    size_t at = keep;
    at += (size_t)snprintf(out + at, cap - at, "\n  <mujoco><compiler meshdir=\"%s\"/></mujoco>\n",
                           dir);
    at += (size_t)snprintf(out + at, cap - at, "%s", txt + keep);
    return at;
  }
  const char* tag_end = strchr(comp, '>');
  if (tag_end == NULL) {
    return (size_t)snprintf(out, cap, "%s", txt);
  }
  const char* existing = strstr(comp, "meshdir=\"");
  if (existing != NULL && existing < tag_end) {
    const char* vstart = existing + strlen("meshdir=\"");
    const char* vend = strchr(vstart, '"');
    if (vend != NULL) {
      size_t at = (size_t)(vstart - txt);
      memcpy(out, txt, at);
      at += (size_t)snprintf(out + at, cap - at, "%s", dir);
      at += (size_t)snprintf(out + at, cap - at, "%s", vend);
      return at;
    }
  }
  size_t keep = (size_t)(tag_end - txt);
  if (keep > 0 && txt[keep - 1] == '/') {
    keep -= 1;  // keep the self-closing slash after the new attribute
  }
  memcpy(out, txt, keep);
  size_t at = keep;
  at += (size_t)snprintf(out + at, cap - at, " meshdir=\"%s\"", dir);
  at += (size_t)snprintf(out + at, cap - at, "%s", txt + keep);
  return at;
}

static int mj_model_text(const char* path, int floating, char** out_text, char* err, int err_cap) {
  FILE* f = fopen(path, "rb");
  if (f == NULL) {
    mj_set_err(err, (size_t)err_cap, "mj_shim: cannot read the model file");
    return -1;
  }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (n <= 0) {
    fclose(f);
    mj_set_err(err, (size_t)err_cap, "mj_shim: the model file is empty");
    return -1;
  }
  char* txt = (char*)malloc((size_t)n + 1);
  if (txt == NULL) {
    fclose(f);
    mj_set_err(err, (size_t)err_cap, "mj_shim: out of memory");
    return -1;
  }
  size_t got = fread(txt, 1, (size_t)n, f);
  txt[got] = 0;
  fclose(f);

  // Unusable geometry goes first: a block referencing an empty mesh would otherwise fail the preparation
  char model_dir[1024];
  mj_dir_of(path, model_dir, sizeof(model_dir));
  char* cleaned = NULL;
  if (mj_drop_unusable_blocks(txt, model_dir, &cleaned) != 0) {
    free(txt);
    mj_set_err(err, (size_t)err_cap, "mj_shim: out of memory");
    return -1;
  }
  free(txt);
  txt = cleaned;

  char* close = txt + strlen(txt);  // where an appended block goes: the end
  char root[256] = {0};
  if (floating) {
    close = strstr(txt, "</robot>");
    if (close == NULL || mj_find_root_link(txt, root, sizeof(root)) != 0) {
      free(txt);
      mj_set_err(err, (size_t)err_cap, "mj_shim: the model file is not a robot with a root link");
      return -1;
    }
  }

  char dir[1024] = {0};
  if (mj_prepare_meshes(path, txt, dir, sizeof(dir), err, err_cap) != 0) {
    free(txt);
    return -1;
  }

  // The floating block goes in BEFORE the meshdir injection (which rewrites the whole model), since after it the block lands outside the root element.
  if (floating) {
    size_t head = (size_t)(close - txt);
    size_t need = head + strlen(root) + strlen(txt) + 512;
    char* with_float = (char*)malloc(need);
    if (with_float == NULL) {
      free(txt);
      mj_set_err(err, (size_t)err_cap, "mj_shim: out of memory");
      return -1;
    }
    snprintf(with_float, need,
             "%.*s  <link name=\"world\"/>\n"
             "  <joint name=\"_mj_float_base\" type=\"floating\">\n"
             "    <parent link=\"world\"/>\n"
             "    <child link=\"%s\"/>\n"
             "  </joint>\n%s",
             (int)head, txt, root, close);
    free(txt);
    txt = with_float;
  }
  size_t cap = strlen(txt) + strlen(dir) + 1024;
  char* out = (char*)malloc(cap);
  if (out == NULL) {
    free(txt);
    mj_set_err(err, (size_t)err_cap, "mj_shim: out of memory");
    return -1;
  }
  size_t at = mj_inject_meshdir(txt, dir, out, cap);
  (void)at;
  free(txt);
  *out_text = out;
  return 0;
}

static int mj_attach_impl(void* scene, const char* urdf, const char* ee, int floating,
                          int* out_robot, char* err, int err_cap) {
  if (scene == NULL || urdf == NULL || out_robot == NULL) {
    return -1;
  }
  // The caller's error sink is not assumed well-formed (a negative cap wraps as size_t, a null buffer with a positive cap writes to address zero), so it is clamped once, at the funnel all three attach paths use.
  static char mj_err_scratch[1];
  if (err == NULL) {
    err = mj_err_scratch;
    err_cap = 0;
  } else if (err_cap < 0) {
    err_cap = 0;
  }
  MjScene* s = (MjScene*)scene;
  if (s->spec != NULL) {
    mj_set_err(err, (size_t)err_cap, "mj_shim: this scene already holds a robot");
    return -1;
  }
  char perr[MJ_ERR_CAP] = {0};
  // Both paths parse from the prepared text: a fixed-base model's meshes may be ASCII too, and a floating one needs the injected world link and joint (mjs_addFreeJoint segfaults on parsed bodies).
  char* text = NULL;
  if (mj_model_text(urdf, floating, &text, err, err_cap) != 0) {
    return -1;
  }
  s->spec = mj_parseXMLString(text, NULL, perr, sizeof(perr));
  if (s->spec != NULL) {
    // The ABI names the end effector by LINK NAME, and a URDF tip link attached by a fixed joint is fused
    // into its parent and deleted as a body by default; fusing removes no joint and no degree of freedom.
    s->spec->compiler.fusestatic = 0;
  }
  if (mj_debug_enabled()) {
    FILE* dbg = fopen("/tmp/mj_model_text_debug.xml", "wb");
    if (dbg != NULL) {
      fwrite(text, 1, strlen(text), dbg);
      fclose(dbg);
      fprintf(stderr, "mj_shim: wrote the parsed model text to /tmp/mj_model_text_debug.xml\n");
    }
  }
  // the model's root link, needed by the slot mapping: a welded root link has no body in the model
  char root_link[256] = {0};
  mj_find_root_link(text, root_link, sizeof(root_link));
  free(text);
  if (s->spec == NULL) {
    if (err != NULL && err_cap > 0) {
      snprintf(err, (size_t)err_cap, "%s", perr);
    }
    return -1;
  }
  for (int i = 0; i < s->n_statics; ++i) {
    if (apply_static(s->spec, &s->statics[i]) != 0) {
      mj_set_err(err, (size_t)err_cap, "mj_shim: a scene static could not be added to the model");
      return -1;
    }
  }
  s->n_statics = 0;

  s->model = mj_compile(s->spec, NULL);
  if (s->model == NULL) {
    char msg[MJ_ERR_CAP];
    snprintf(msg, sizeof(msg), "mj_shim: compiling the model failed: %s", mjs_getError(s->spec));
    mj_set_err(err, (size_t)err_cap, msg);
    return -1;
  }
  // SD_MJ_DEBUG prints the model's joint table: which joints came out, of which type, where slots live
  if (mj_debug_enabled()) {
    fprintf(stderr, "mj_shim: model %s: nq=%d nv=%d njnt=%d nbody=%d ngeom=%d\n", urdf,
            (int)s->model->nq, (int)s->model->nv, (int)s->model->njnt, (int)s->model->nbody,
            (int)s->model->ngeom);
    for (int j = 0; j < s->model->njnt; ++j) {
      fprintf(stderr, "mj_shim:   jnt[%d] %-24s type=%d qposadr=%d dofadr=%d\n", j,
              mj_id2name(s->model, mjOBJ_JOINT, j), (int)s->model->jnt_type[j],
              (int)s->model->jnt_qposadr[j], (int)s->model->jnt_dofadr[j]);
    }
    for (int b = 0; b < s->model->nbody && b < 4; ++b) {
      fprintf(stderr, "mj_shim:   body[%d] %-24s mass=%.4f\n", b,
              mj_id2name(s->model, mjOBJ_BODY, b), s->model->body_mass[b]);
    }
  }
  s->data = mj_makeData(s->model);

  // Two collision classes give exactly the engine's own contract: contact between a link and the scene's statics, and nothing between a robot's own links.
  mj_apply_contact_policy(s->model);

  MjRobot* r = (MjRobot*)calloc(1, sizeof(MjRobot));
  if (r == NULL) {
    mj_set_err(err, (size_t)err_cap, "mj_shim: out of memory");
    return -1;
  }
  r->scene = s;
  r->n_joints = s->model->njnt;
  r->nv = r->n_joints;  // one DOF per joint: a hinge carries a single velocity
  r->qpos_adr = (int*)calloc((size_t)(r->n_joints > 0 ? r->n_joints : 1), sizeof(int));
  r->dof_adr = (int*)calloc((size_t)(r->n_joints > 0 ? r->n_joints : 1), sizeof(int));
  r->slot_of_body = (int*)calloc((size_t)(s->model->nbody > 0 ? s->model->nbody : 1), sizeof(int));
  r->pen = (double*)calloc((size_t)(s->model->nbody > 0 ? s->model->nbody : 1), sizeof(double));
  r->pen_n = (double*)calloc((size_t)(s->model->nbody > 0 ? s->model->nbody : 1) * 3, sizeof(double));
  r->pen_p = (double*)calloc((size_t)(s->model->nbody > 0 ? s->model->nbody : 1) * 3, sizeof(double));
  r->pen_w = (double*)calloc((size_t)(s->model->nbody > 0 ? s->model->nbody : 1) * 3, sizeof(double));
  r->pen_wsum = (double*)calloc((size_t)(s->model->nbody > 0 ? s->model->nbody : 1), sizeof(double));
  {
    // the readout is sized by the V-side slot count, which mj_resolve fixes
    size_t slots = (size_t)(s->model->njnt + 1);
    r->fc_filt = (float*)calloc(slots * 3, sizeof(float));
    r->fw_filt = (float*)calloc(slots * 6, sizeof(float));
  }
  r->n_links = s->model->nbody;
  r->ee_body = -1;
  if (ee != NULL) {
    r->ee_body = mj_name2id(s->model, mjOBJ_BODY, ee);
  }
  r->base_dofs = floating ? 6 : 0;
  r->base_qpos_adr = -1;
  r->base_dof_adr = -1;
  if (floating) {
    for (int j = 0; j < s->model->njnt; ++j) {
      if (s->model->jnt_type[j] == mjJNT_FREE) {
        r->base_qpos_adr = (int)s->model->jnt_qposadr[j];
        r->base_dof_adr = (int)s->model->jnt_dofadr[j];
        break;
      }
    }
    if (r->base_qpos_adr < 0) {
      mj_set_err(err, (size_t)err_cap, "mj_shim: the model was asked to float but has no free joint");
      return -1;
    }
  }
  if (mj_resolve(s, r, root_link) != 0) {
    mj_set_err(err, (size_t)err_cap, "mj_shim: the model has no joints to drive");
  }
  s->robot = r;
  *out_robot = 0;
  return 0;
}

int eng_robot_attach(void* scene, const char* urdf, const char* ee, void** out, char* err, int cap) {
  if (out == NULL) {
    return -1;
  }
  *out = NULL;
  int idx = 0;
  if (mj_attach_impl(scene, urdf, ee, /*floating=*/0, &idx, err, cap) != 0) {
    return -1;
  }
  *out = ((MjScene*)scene)->robot;
  return 0;
}

int eng_robot_attach_ex(void* scene, const char* urdf, const char* ee, int weld_root,
                       double voxel, void** out, char* err, int cap) {
  (void)voxel;  // MuJoCo's own collision geometry replaces the SDF voxel choice
  if (weld_root == 0) {
    return eng_robot_attach_floating(scene, urdf, ee, out, err, cap);
  }
  return eng_robot_attach(scene, urdf, ee, out, err, cap);
}

int eng_robot_attach_floating(void* scene, const char* urdf, const char* ee, void** out,
                             char* err, int cap) {
  if (out == NULL) {
    return -1;
  }
  *out = NULL;
  int idx = 0;
  if (mj_attach_impl(scene, urdf, ee, /*floating=*/1, &idx, err, cap) != 0) {
    return -1;
  }
  *out = ((MjScene*)scene)->robot;
  return 0;
}

void eng_robot_free(void* robot) {
  // the robot's storage is owned by its scene, which frees it in eng_scene_free
  (void)robot;
}

// mj_gl_make_current gives this process an offscreen OpenGL context and makes it current: MuJoCo's
// renderer draws into whatever context is current and creates none of its own (mjr_makeContext dies
// inside glGetString without one), and the legacy profile is the path mjr's drawing takes.
static CGLContextObj mj_gl_ctx = NULL;
static int mj_gl_make_current(char* err, int cap) {
  if (mj_gl_ctx != NULL) {
    CGLSetCurrentContext(mj_gl_ctx);
    return 0;
  }
  CGLPixelFormatAttribute attrs[] = {
      kCGLPFAAccelerated,
      kCGLPFAColorSize,
      (CGLPixelFormatAttribute)24,
      kCGLPFADepthSize,
      (CGLPixelFormatAttribute)24,
      (CGLPixelFormatAttribute)0,
  };
  CGLPixelFormatObj pix = NULL;
  GLint npix = 0;
  CGLError e = CGLChoosePixelFormat(attrs, &pix, &npix);
  if (e != kCGLNoError || pix == NULL) {
    snprintf(err, (size_t)cap, "mj_shim: no OpenGL pixel format for offscreen rendering (%s)",
             CGLErrorString(e));
    return -1;
  }
  e = CGLCreateContext(pix, NULL, &mj_gl_ctx);
  CGLDestroyPixelFormat(pix);
  if (e != kCGLNoError || mj_gl_ctx == NULL) {
    snprintf(err, (size_t)cap, "mj_shim: cannot create a GL context for rendering (%s)",
             CGLErrorString(e));
    return -1;
  }
  CGLSetCurrentContext(mj_gl_ctx);
  return 0;
}

// eng_scene_render draws one frame and hands back packed RGB8 (width * height * 3, top row first); a null
// lookat tracks the robot's CoM, azimuth/elevation in degrees, distance in metres. Built on first use.
int eng_scene_render(void* scene, int width, int height, const float* lookat, float distance,
                    float azimuth, float elevation, unsigned char* rgb, int rgb_cap) {
  if (scene == NULL || rgb == NULL || width <= 0 || height <= 0) {
    return -1;
  }
  MjScene* s = (MjScene*)scene;
  if (s->model == NULL || s->data == NULL) {
    snprintf(s->render_error, sizeof(s->render_error), "mj_shim: this scene has no robot yet");
    return -1;
  }
  if ((long)rgb_cap < (long)width * (long)height * 3) {
    snprintf(s->render_error, sizeof(s->render_error),
             "mj_shim: a %dx%d frame needs %ld bytes, %d given", width, height,
             (long)width * (long)height * 3, rgb_cap);
    return -1;
  }
  mjModel* m = s->model;
  mjData* d = s->data;
  if (s->vis_ready == 0) {
    if (mj_gl_make_current(s->render_error, sizeof(s->render_error)) != 0) {
      return -1;
    }
    mjv_defaultCamera(&s->cam);
    mjv_defaultOption(&s->opt);
    mjv_defaultScene(&s->vis);
    mjr_defaultContext(&s->con);
    mjv_makeScene(m, &s->vis, 4000);  // room for the robot, the statics and the arrows
    mjr_makeContext(m, &s->con, mjFONTSCALE_150);
    // draw into the offscreen buffer: the default target is the window buffer, which a program with no window cannot read back
    mjr_setBuffer(mjFB_OFFSCREEN, &s->con);
    s->vis_ready = 1;
  }
  if (s->rgb_w != width || s->rgb_h != height) {
    unsigned char* grown = (unsigned char*)realloc(s->rgb, (size_t)width * (size_t)height * 3);
    if (grown == NULL) {
      snprintf(s->render_error, sizeof(s->render_error), "mj_shim: out of memory for a frame");
      return -1;
    }
    s->rgb = grown;
    s->rgb_w = width;
    s->rgb_h = height;
    mjr_resizeOffscreen(width, height, &s->con);
  }
  // where the camera looks: the caller's point, or the robot's own centre of mass (which follows the machine)
  mjtNum target[3];
  if (lookat != NULL) {
    target[0] = lookat[0];
    target[1] = lookat[1];
    target[2] = lookat[2];
  } else {
    mj_forward(m, d);
    // body 1 is the robot's root (the world body is 0): its subtree's centre of mass is the machine's
    target[0] = d->subtree_com[3 * 1 + 0];
    target[1] = d->subtree_com[3 * 1 + 1];
    target[2] = d->subtree_com[3 * 1 + 2];
  }
  s->cam.type = mjCAMERA_FREE;
  for (int i = 0; i < 3; ++i) {
    s->cam.lookat[i] = target[i];
  }
  s->cam.distance = distance > 0.0f ? (double)distance : (double)m->stat.extent * 4.0;
  s->cam.azimuth = (double)azimuth;
  s->cam.elevation = (double)elevation;
  mjrRect rect = {0, 0, width, height};
  mjv_updateScene(m, d, &s->opt, NULL, &s->cam, mjCAT_ALL, &s->vis);
  if (mj_debug_enabled()) {
    fprintf(stderr,
            "mj_shim: render %dx%d: ngeom=%d nlight=%d cam lookat=[%.3f %.3f %.3f] dist=%.2f "
            "az=%.1f el=%.1f | model extent=%.3f center=[%.3f %.3f %.3f]\n",
            width, height, s->vis.ngeom, s->vis.nlight, s->cam.lookat[0], s->cam.lookat[1],
            s->cam.lookat[2], s->cam.distance, s->cam.azimuth, s->cam.elevation, m->stat.extent,
            m->stat.center[0], m->stat.center[1], m->stat.center[2]);
    // what the visualisation actually holds: a populated scene that renders black is a material or category problem
    for (int i = 0; i < s->vis.ngeom && i < 6; ++i) {
      const mjvGeom* g = s->vis.geoms + i;
      fprintf(stderr, "mj_shim:   vis geom[%d] type=%d cat=%d rgba=[%.2f %.2f %.2f %.2f]\n", i,
              (int)g->type, (int)g->category, g->rgba[0], g->rgba[1], g->rgba[2], g->rgba[3]);
    }
  }
  mjr_render(rect, &s->vis, &s->con);
  mjr_readPixels(s->rgb, NULL, rect, &s->con);
  // the GL frame comes back bottom-up, so the rows are flipped on the way out (every image format expects the top row first)
  const int stride = width * 3;
  for (int y = 0; y < height / 2; ++y) {
    unsigned char* top = s->rgb + (size_t)y * stride;
    unsigned char* bot = s->rgb + (size_t)(height - 1 - y) * stride;
    for (int i = 0; i < stride; ++i) {
      unsigned char tmp = top[i];
      top[i] = bot[i];
      bot[i] = tmp;
    }
  }
  memcpy(rgb, s->rgb, (size_t)stride * (size_t)height);
  return 0;
}

const char* eng_scene_render_error(void* scene) {
  if (scene == NULL) {
    return "mj_shim: null scene";
  }
  return ((MjScene*)scene)->render_error;
}

const char* eng_robot_last_error(void* robot) {
  if (robot == NULL) {
    return "null robot";
  }
  return ((MjRobot*)robot)->last_error;
}

int eng_robot_num_dofs(void* robot) {
  if (robot == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  return r->base_dofs + r->n_joints;
}

// ---- the base's state, in this ABI's layout --------------------------------
// A floating base is a rotation VECTOR leading every state vector here, while MuJoCo's free joint is a
// quaternion whose angular velocity is in the BODY frame; the linear part passes through unchanged.

static void mj_rotvec_to_quat(const float* rv, mjtNum* quat) {
  mjtNum v[3] = {rv[0], rv[1], rv[2]};
  mjtNum ang = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (ang < 1e-12) {
    quat[0] = 1.0;
    quat[1] = 0.0;
    quat[2] = 0.0;
    quat[3] = 0.0;
    return;
  }
  mjtNum axis[3] = {v[0] / ang, v[1] / ang, v[2] / ang};
  mju_axisAngle2Quat(quat, axis, ang);
}

static void mj_quat_to_rotvec(const mjtNum* quat, float* rv) {
  mjtNum vel[3];
  mju_quat2Vel(vel, quat, 1.0);  // axis * angle, i.e. the rotation vector
  rv[0] = (float)vel[0];
  rv[1] = (float)vel[1];
  rv[2] = (float)vel[2];
}

// world-frame omega -> the body frame the free joint stores it in
static void mj_omega_world_to_body(const mjtNum* quat, const float* omega_world, mjtNum* out) {
  mjtNum R[9];
  mju_quat2Mat(R, quat);
  for (int i = 0; i < 3; ++i) {
    out[i] = R[0 * 3 + i] * omega_world[0] + R[1 * 3 + i] * omega_world[1] +
             R[2 * 3 + i] * omega_world[2];
  }
}

static void mj_omega_body_to_world(const mjtNum* quat, const mjtNum* omega_body, float* out) {
  mjtNum R[9];
  mju_quat2Mat(R, quat);
  for (int i = 0; i < 3; ++i) {
    out[i] = (float)(R[i * 3 + 0] * omega_body[0] + R[i * 3 + 1] * omega_body[1] +
                     R[i * 3 + 2] * omega_body[2]);
  }
}

// ----------------------------------------------------------- state in/out --

int eng_robot_set_state(void* robot, const float* q, const float* v) {
  if (robot == NULL || q == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjData* d = r->scene->data;
  if (r->base_dofs == 6) {
    const int b = r->base_qpos_adr;
    for (int i = 0; i < 3; ++i) {
      d->qpos[b + i] = (mjtNum)q[i];
    }
    mj_rotvec_to_quat(q + 3, d->qpos + b + 3);
  }
  const int qo = r->base_dofs;  // the joints follow the base in this layout
  for (int j = 0; j < r->n_joints; ++j) {
    d->qpos[r->qpos_adr[j]] = (mjtNum)q[qo + j];
  }
  // NO forward pass between the position and velocity writes: the omegas below read the base quaternion
  // straight out of qpos, and the final mj_forward is the state the caller reads back (see docs/PERF_AUDIT.md).
  if (v != NULL) {
    if (r->base_dofs == 6) {
      const int bd = r->base_dof_adr;
      for (int i = 0; i < 3; ++i) {
        d->qvel[bd + i] = (mjtNum)v[i];
      }
      mjtNum wb[3];
      mj_omega_world_to_body(d->qpos + r->base_qpos_adr + 3, v + 3, wb);
      for (int i = 0; i < 3; ++i) {
        d->qvel[bd + 3 + i] = wb[i];
      }
    }
    for (int j = 0; j < r->n_joints; ++j) {
      d->qvel[r->dof_adr[j]] = (mjtNum)v[qo + j];
    }
  }
  mj_forward(r->scene->model, d);
  return 0;
}

int eng_robot_get_q(void* robot, float* q) {
  if (robot == NULL || q == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjData* d = r->scene->data;
  if (r->base_dofs == 6) {
    const int b = r->base_qpos_adr;
    for (int i = 0; i < 3; ++i) {
      q[i] = (float)d->qpos[b + i];
    }
    mj_quat_to_rotvec(d->qpos + b + 3, q + 3);
  }
  const int qo = r->base_dofs;
  for (int j = 0; j < r->n_joints; ++j) {
    q[qo + j] = (float)d->qpos[r->qpos_adr[j]];
  }
  return 0;
}

int eng_robot_get_v(void* robot, float* v) {
  if (robot == NULL || v == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjData* d = r->scene->data;
  if (r->base_dofs == 6) {
    const int bd = r->base_dof_adr;
    for (int i = 0; i < 3; ++i) {
      v[i] = (float)d->qvel[bd + i];
    }
    mj_omega_body_to_world(d->qpos + r->base_qpos_adr + 3, d->qvel + bd + 3, v + 3);
  }
  const int qo = r->base_dofs;
  for (int j = 0; j < r->n_joints; ++j) {
    v[qo + j] = (float)d->qvel[r->dof_adr[j]];
  }
  return 0;
}

int eng_robot_ee_pos(void* robot, float* pos) {
  if (robot == NULL || pos == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  if (r->ee_body < 0) {
    return -1;
  }
  const mjtNum* p = r->scene->data->xpos + 3 * r->ee_body;
  pos[0] = (float)p[0];
  pos[1] = (float)p[1];
  pos[2] = (float)p[2];
  return 0;
}

// eng_robot_step applies the joint torques and advances the engine: n_sub steps of dt each, the model's
// timestep set to dt (spreading dt over the substeps reads as n_sub times too much force).
// mj_snapshot_penetration records, per body, the contact its commanded pose creates BEFORE the step.
static void mj_snapshot_penetration(MjRobot* r) {
  mjModel* m = r->scene->model;
  mjData* d = r->scene->data;
  int nbody = (int)m->nbody;
  for (int b = 0; b < nbody; ++b) {
    r->pen[b] = 0.0;
    r->pen_wsum[b] = 0.0;
    for (int i = 0; i < 3; ++i) {
      r->pen_w[3 * b + i] = 0.0;
    }
  }
  mj_forward(m, d);
  for (int c = 0; c < d->ncon; ++c) {
    const mjContact* ct = d->contact + c;
    double pen = (double)(-ct->dist);
    if (pen <= 0.0) {
      continue;
    }
    for (int side = 0; side < 2; ++side) {
      int g = ct->geom[side];
      if (g < 0) {
        continue;
      }
      int b = (int)m->geom_bodyid[g];
      if (b <= 0 || b >= nbody) {
        continue;
      }
      // Every contact the body has, at its own overlap depth: the centroid of the distribution this model implies, taken before the deepest test below
      double w = pen;
      r->pen_wsum[b] += w;
      for (int i = 0; i < 3; ++i) {
        r->pen_w[3 * b + i] += w * (double)ct->pos[i];
      }
      if (pen <= r->pen[b]) {
        continue;
      }
      r->pen[b] = pen;
      double sgn = (side == 1) ? 1.0 : -1.0;
      for (int i = 0; i < 3; ++i) {
        r->pen_n[3 * b + i] = sgn * (double)ct->frame[0 * 3 + i];
        r->pen_p[3 * b + i] = (double)ct->pos[i];
      }
    }
  }
}

int eng_robot_step(void* robot, const float* tau, int n_sub, double dt) {
  MjRobot* rep = (MjRobot*)robot;
  if (robot == NULL || tau == NULL || n_sub <= 0 || dt < 0.0) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjModel* m = r->scene->model;
  mjData* d = r->scene->data;
  // tau leads with the base rows when the root floats (its callers zero them: a free base has no actuators)
  for (int j = 0; j < r->n_joints; ++j) {
    d->qfrc_applied[r->dof_adr[j]] = (mjtNum)tau[r->base_dofs + j];
  }
  if (dt > 0.0) {
    if (mj_penalty_report(rep)) {
      mj_snapshot_penetration(r);
    }
    // the readout's low-pass advances by the simulated time this step covers
    r->last_dt = dt * (double)n_sub;
    m->opt.timestep = dt;
    for (int i = 0; i < n_sub; ++i) {
      mj_step(m, d);
    }
  } else {
    // dt == 0 is the contract's query-refresh step: the world does not advance, the readout is refreshed
    if (mj_penalty_report(rep)) {
      mj_snapshot_penetration(r);
    }
    mj_forward(m, d);
  }
  return 0;
}

// The engine's contact readout, per body. The aggregate pair (eng_robot_contact_force/torque) is the
// solver's applied external force, whose moment reference point differs from the replaced engine's.
// mj_contact_tables fills the per-link tables from the contacts themselves (cfrc_ext reads zero for a
// resting contact): mj_contactForce returns the 6D force:torque in the CONTACT frame, whose first axis
// is the normal from geom[0] to geom[1] and whose force acts on the body owning geom[1]; the moment is
// about each body's own ORIGIN (cop_x = ty/fz there), and the entries are in MUJOCO's body order while
// the V-side tree is not — the counts agree, the order does not.
// mj_penalty_report selects the reading: the default is the solver's own force, the load a steady contact
// carries; MJ_CONTACT_REPORT=penalty gives k times the overlap the caller's pose creates, which
// reproduces the replaced engine's penalty contact but under-reports a held load (25 N reads 0.65 N).
static int mj_penalty_report(MjRobot* r) {
  if (r != NULL && r->report_set) {
    return r->report_penalty;
  }
  const char* v = getenv("MJ_CONTACT_REPORT");
  return v != NULL && strcmp(v, "penalty") == 0;
}

// mj_contact_stiffness is the penalty report's stiffness in N per metre of overlap: the project's own
// 5e7 Pa/m over a 0.013 m2 sole, which stands 327 N of weight at 0.25 mm. MJ_CONTACT_STIFFNESS moves it.
static mjtNum mj_contact_stiffness(MjRobot* r) {
  if (r != NULL && r->report_set) {
    return (mjtNum)r->report_stiffness;
  }
  return (mjtNum)mj_env_num("MJ_CONTACT_STIFFNESS", 650000.0);
}

// MJ_CONTACT_FILTER is an optional low-pass on the readout, in seconds; zero disables it, the refresh step
// never filters, and it is off by default because the contact's own reference time already band-limits.
#define MJ_CONTACT_FILTER 0.0

static double mj_filter_alpha(const MjRobot* r) {
  if (r->last_dt <= 0.0) {
    return 1.0;
  }
  double tau = mj_env_num("MJ_CONTACT_FILTER", MJ_CONTACT_FILTER);
  if (tau <= 0.0) {
    return 1.0;
  }
  double a = r->last_dt / tau;
  return a > 1.0 ? 1.0 : a;
}

// mj_slot_of_body answers which slot a MuJoCo body's contacts report into: a body with a node of the
// V-side tree reports into that node's slot (slot s holds node s+1, the last slot the root), and a body
// with none — attached by a fixed joint, such as a fingertip pad — into the nearest jointed ancestor's slot.
static int mj_slot_of_body(const MjRobot* r, const mjModel* m, int b) {
  int n = r->n_links;
  for (int p = b; p > 0; p = (int)m->body_parentid[p]) {
    for (int node = 0; node <= r->n_joints; ++node) {
      if (r->slot_of_body[node] == p) {
        return node == 0 ? n - 1 : node - 1;
      }
    }
  }
  return -1;
}

static int mj_contact_tables(MjRobot* r, float* forces3, float* wrenches6, int n_max) {
  mjModel* m = r->scene->model;
  mjData* d = r->scene->data;
  int n = r->n_links < n_max ? r->n_links : n_max;
  if (forces3 != NULL) {
    for (int i = 0; i < 3 * n; ++i) {
      forces3[i] = 0.0f;
    }
  }
  if (wrenches6 != NULL) {
    for (int i = 0; i < 6 * n; ++i) {
      wrenches6[i] = 0.0f;
    }
  }
  // accumulated per MuJoCo body first and translated into V-side slots at the end (the orders differ)
  int nbody = (int)m->nbody;
  float* fw_all = (float*)calloc((size_t)nbody * 3, sizeof(float));
  float* tw_all = wrenches6 != NULL ? (float*)calloc((size_t)nbody * 6, sizeof(float)) : NULL;
  int* ncon_of_body = (int*)calloc((size_t)nbody, sizeof(int));
  if (fw_all == NULL || (wrenches6 != NULL && tw_all == NULL) || ncon_of_body == NULL) {
    free(fw_all);
    free(tw_all);
    free(ncon_of_body);
    return -1;
  }
  if (mj_penalty_report(r)) {
    for (int c = 0; c < d->ncon; ++c) {
      for (int side = 0; side < 2; ++side) {
        int g = d->contact[c].geom[side];
        if (g < 0) {
          continue;
        }
        int b = (int)m->geom_bodyid[g];
        if (b > 0 && b < nbody) {
          ncon_of_body[b]++;
        }
      }
    }
  }
  // The penalty report's normal force: k times the body's deepest overlap, along the normal that pushes the
  // body out, applied at the patch centroid of its contacts (not the deepest point, a sole corner).
  if (mj_penalty_report(r)) {
    mjtNum k = mj_contact_stiffness(r);
    for (int b = 1; b < nbody; ++b) {
      double pen = r->pen[b];
      if (pen <= 0.0) {
        continue;
      }
      mjtNum fn = k * (mjtNum)pen;
      if (ncon_of_body[b] > 1) {
        fn /= (mjtNum)ncon_of_body[b];
      }
      mjtNum F[3];
      for (int i = 0; i < 3; ++i) {
        F[i] = fn * (mjtNum)r->pen_n[3 * b + i];
      }
      mjtNum rr[3];
      if (r->pen_wsum[b] > 0.0) {
        for (int i = 0; i < 3; ++i) {
          rr[i] = (mjtNum)(r->pen_w[3 * b + i] / r->pen_wsum[b]) - d->xpos[3 * b + i];
        }
      } else {
        for (int i = 0; i < 3; ++i) {
          rr[i] = (mjtNum)r->pen_p[3 * b + i] - d->xpos[3 * b + i];
        }
      }
      mjtNum M[3] = {rr[1] * F[2] - rr[2] * F[1], rr[2] * F[0] - rr[0] * F[2],
                     rr[0] * F[1] - rr[1] * F[0]};
      for (int i = 0; i < 3; ++i) {
        fw_all[3 * b + i] += (float)F[i];
        if (tw_all != NULL) {
          tw_all[6 * b + i] += (float)F[i];
          tw_all[6 * b + 3 + i] += (float)M[i];
        }
      }
    }
  }
  for (int c = 0; c < d->ncon; ++c) {
    const mjContact* ct = d->contact + c;
    mjtNum f6[6];
    mj_contactForce(m, d, c, f6);
    if (mj_debug_enabled()) {
      const char* g0 = ct->geom[0] >= 0 ? mj_id2name(m, mjOBJ_GEOM, ct->geom[0]) : "-";
      const char* g1 = ct->geom[1] >= 0 ? mj_id2name(m, mjOBJ_GEOM, ct->geom[1]) : "-";
      fprintf(stderr,
              "mj_shim: con[%d] %s/%s geom=%s|%s dist=%.6f pos=[%.4f %.4f %.4f] n=[%.3f %.3f %.3f] "
              "f6=[%.3f %.3f %.3f]\n",
              c, mj_id2name(m, mjOBJ_BODY, (int)m->geom_bodyid[ct->geom[0]]),
              mj_id2name(m, mjOBJ_BODY, (int)m->geom_bodyid[ct->geom[1]]), g0, g1, ct->dist,
              ct->pos[0], ct->pos[1], ct->pos[2], ct->frame[0], ct->frame[1], ct->frame[2], f6[0],
              f6[1], f6[2]);
    }
    // The reported normal force is zero under the penalty report (the snapshot pass added it); what remains
    // here is the solver's own friction on the two tangent axes, dropped when the body overlaps nothing.
    mjtNum fn_report = f6[0];
    if (mj_penalty_report(r)) {
      fn_report = 0.0;
    }
    // the contact frame stores its basis vectors as ROWS (row 0 the normal), so contact frame -> world is the transposed multiply, Fw_i = sum_j frame[j][i] * f_j
    for (int side = 0; side < 2; ++side) {
      int g = ct->geom[side];
      if (g < 0) {
        continue;
      }
      int b = (int)m->geom_bodyid[g];
      if (b <= 0 || b >= nbody) {
        continue;  // the scene's statics take no entry of their own
      }
      mjtNum fn = fn_report;
      // the solver's tangents are friction, unbounded while the surfaces are apart
      mjtNum ft1 = fn_report > 0.0 ? f6[1] : 0.0;
      mjtNum ft2 = fn_report > 0.0 ? f6[2] : 0.0;
      if (mj_penalty_report(r) && ncon_of_body[b] > 1) {
        fn /= (mjtNum)ncon_of_body[b];
        ft1 /= (mjtNum)ncon_of_body[b];
        ft2 /= (mjtNum)ncon_of_body[b];
      }
      mjtNum f3[3] = {fn, ft1, ft2};
      mjtNum t3[3] = {f6[3], f6[4], f6[5]};
      mjtNum sgn = (side == 1) ? 1.0 : -1.0;  // one side negates, by the third law
      mjtNum F[3], T[3];
      for (int i = 0; i < 3; ++i) {
        F[i] = sgn * (ct->frame[0 * 3 + i] * f3[0] + ct->frame[1 * 3 + i] * f3[1] +
                      ct->frame[2 * 3 + i] * f3[2]);
        T[i] = sgn * (ct->frame[0 * 3 + i] * t3[0] + ct->frame[1 * 3 + i] * t3[1] +
                      ct->frame[2 * 3 + i] * t3[2]);
      }
      if (mj_debug_enabled() && c == 0 && side == 1) {
        fprintf(stderr, "mj_shim:   first contact on %s: F=[%.2f %.2f %.2f]\n",
                mj_id2name(m, mjOBJ_BODY, b), F[0], F[1], F[2]);
        // Once per process: how far this body's own MESH vertices reach in world x (a hull fills the mesh's concavities)
        static int hull_dumped = 0;
        // taken at the first contact that overlaps: before it they are the freshly compiled model's zeros
        static int body_dumped = 0;
        if (body_dumped == 0 && ct->dist < 0.0) {
          body_dumped = 1;
          for (int gi = 0; gi < (int)m->ngeom; ++gi) {
            int bb = (int)m->geom_bodyid[gi];
            fprintf(stderr,
                    "mj_shim:   geom[%d] body=%-12s type=%d pos=[%.4f %.4f %.4f] aabb=[%.3f %.3f "
                    "%.3f]\n",
                    gi, mj_id2name(m, mjOBJ_BODY, bb), (int)m->geom_type[gi],
                    d->geom_xpos[3 * gi + 0], d->geom_xpos[3 * gi + 1], d->geom_xpos[3 * gi + 2],
                    m->geom_aabb[6 * gi + 3], m->geom_aabb[6 * gi + 4], m->geom_aabb[6 * gi + 5]);
          }
        }
        if (hull_dumped == 0) {
          hull_dumped = 1;
          double mesh_min_x = 1e30;
          double near_max_x = -1e30;
          int verts = 0;
          int near = 0;
          for (int gi = 0; gi < m->ngeom; ++gi) {
            if ((int)m->geom_bodyid[gi] != b || m->geom_type[gi] != mjGEOM_MESH) {
              continue;
            }
            int mid = (int)m->geom_dataid[gi];
            if (mid < 0) {
              continue;
            }
            int v0 = (int)m->mesh_vertadr[mid];
            int vn = (int)m->mesh_vertnum[mid];
            for (int v = v0; v < v0 + vn; ++v) {
              double lx = m->mesh_vert[3 * v + 0], ly = m->mesh_vert[3 * v + 1],
                     lz = m->mesh_vert[3 * v + 2];
              double wx = d->geom_xpos[3 * gi + 0] + d->geom_xmat[9 * gi + 0] * lx +
                          d->geom_xmat[9 * gi + 1] * ly + d->geom_xmat[9 * gi + 2] * lz;
              double wy = d->geom_xpos[3 * gi + 1] + d->geom_xmat[9 * gi + 3] * lx +
                          d->geom_xmat[9 * gi + 4] * ly + d->geom_xmat[9 * gi + 5] * lz;
              double wz = d->geom_xpos[3 * gi + 2] + d->geom_xmat[9 * gi + 6] * lx +
                          d->geom_xmat[9 * gi + 7] * ly + d->geom_xmat[9 * gi + 8] * lz;
              if (wx < mesh_min_x) {
                mesh_min_x = wx;
              }
              // near the contact: does the MESH reach as far along +x as the contact claims?
              if (fabs(wy - ct->pos[1]) < 0.03 && fabs(wz - ct->pos[2]) < 0.03) {
                if (wx > near_max_x) {
                  near_max_x = wx;
                }
                near++;
              }
              verts++;
            }
          }
          if (verts > 0) {
            fprintf(stderr,
                    "mj_shim:   %s mesh: %d verts, smallest world x = %.4f; %d verts within "
                    "3 cm of the contact's y,z, largest world x there = %.4f (contact x %.4f)\n",
                    mj_id2name(m, mjOBJ_BODY, b), verts, mesh_min_x, near, near_max_x, ct->pos[0]);
          }
        }
      }
      mjtNum rr[3] = {ct->pos[0] - d->xpos[3 * b + 0], ct->pos[1] - d->xpos[3 * b + 1],
                      ct->pos[2] - d->xpos[3 * b + 2]};
      mjtNum M[3] = {rr[1] * F[2] - rr[2] * F[1] + T[0], rr[2] * F[0] - rr[0] * F[2] + T[1],
                     rr[0] * F[1] - rr[1] * F[0] + T[2]};
      for (int i = 0; i < 3; ++i) {
        fw_all[3 * b + i] += (float)F[i];
        if (tw_all != NULL) {
          tw_all[6 * b + i] += (float)F[i];
          tw_all[6 * b + 3 + i] += (float)M[i];
        }
      }
    }
  }
  // the V side's layout: slot s holds node s+1, the LAST slot the root; a body's contacts land in its own
  // slot or, for a body that is not a node, in the slot of the link that carries it.
  for (int b = 1; b < nbody; ++b) {
    int s = mj_slot_of_body(r, m, b);
    if (s < 0 || s >= n) {
      continue;
    }
    for (int i = 0; i < 3; ++i) {
      // each table is written only when the caller asked for it (a null buffer means the other readout)
      if (forces3 != NULL) {
        forces3[3 * s + i] += fw_all[3 * b + i];
      }
      if (wrenches6 != NULL && tw_all != NULL) {
        wrenches6[6 * s + i] += tw_all[6 * b + i];
        wrenches6[6 * s + 3 + i] += tw_all[6 * b + 3 + i];
      }
    }
  }
  free(fw_all);
  free(tw_all);
  free(ncon_of_body);
  // The readout's low-pass: a constraint contact delivers its correction inside one tick, so the first tick reads an impulse, not a force.
  double alpha = mj_filter_alpha(r);
  if (forces3 != NULL) {
    for (int i = 0; i < 3 * n; ++i) {
      r->fc_filt[i] = (float)(r->fc_filt[i] + alpha * ((double)forces3[i] - r->fc_filt[i]));
      forces3[i] = r->fc_filt[i];
    }
  }
  if (wrenches6 != NULL) {
    for (int i = 0; i < 6 * n; ++i) {
      r->fw_filt[i] = (float)(r->fw_filt[i] + alpha * ((double)wrenches6[i] - r->fw_filt[i]));
      wrenches6[i] = r->fw_filt[i];
    }
  }
  if (mj_debug_enabled() && forces3 != NULL) {
    // slot-indexed: the values are per V-side slot, so naming them with a MuJoCo body id would mislabel every row
    for (int s = 0; s < n; ++s) {
      int node = (s == n - 1) ? 0 : s + 1;
      int src = r->slot_of_body[node];
      const char* nm =
          (src > 0 && src < nbody) ? mj_id2name(m, mjOBJ_BODY, src) : "root(no slot of its own)";
      fprintf(stderr, "mj_shim: slot[%d] node=%d %-14s f=[%.2f %.2f %.2f]\n", s, node, nm,
              forces3[3 * s + 0], forces3[3 * s + 1], forces3[3 * s + 2]);
    }
  }
  return r->n_links;
}

int eng_robot_link_contact_forces(void* robot, float* out, int n_max) {
  if (robot == NULL || out == NULL) {
    return -1;
  }
  return mj_contact_tables((MjRobot*)robot, out, NULL, n_max);
}

// The per-foot pressure-distribution sense: the same contacts with the moment about each body's own origin, so cop_x = ty/fz there is a pressure point.
int eng_robot_link_contact_wrenches(void* robot, float* out, int n_max) {
  if (robot == NULL || out == NULL) {
    return -1;
  }
  return mj_contact_tables((MjRobot*)robot, NULL, out, n_max);
}

int eng_robot_contact_force(void* robot, float* out) {
  if (robot == NULL || out == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjData* d = r->scene->data;
  mjtNum f[3] = {0, 0, 0};
  for (int b = 0; b < r->n_links; ++b) {
    const mjtNum* fb = d->cfrc_ext + 6 * b;
    if (b == 0) {
      continue;  // the world body carries no contact of its own
    }
    f[0] += fb[0];
    f[1] += fb[1];
    f[2] += fb[2];
  }
  out[0] = (float)f[0];
  out[1] = (float)f[1];
  out[2] = (float)f[2];
  return 0;
}

int eng_robot_contact_torque(void* robot, float* out) {
  if (robot == NULL || out == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjData* d = r->scene->data;
  mjtNum t[3] = {0, 0, 0};
  for (int b = 1; b < r->n_links; ++b) {
    const mjtNum* fb = d->cfrc_ext + 6 * b;
    t[0] += fb[3];
    t[1] += fb[4];
    t[2] += fb[5];
  }
  out[0] = (float)t[0];
  out[1] = (float)t[1];
  out[2] = (float)t[2];
  return 0;
}

// ------------------------------------------------------- not yet ported ----
// Part of the contract the V side binds: these fail loudly instead of returning a plausible zero.

// Zero Coulomb/stiction friction, keep the URDF viscous damping (MuJoCo reads friction off the geoms).
int eng_robot_keep_viscous_only(void* robot) {
  if (robot == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjModel* m = r->scene->model;
  for (int g = 0; g < m->ngeom; ++g) {
    m->geom_friction[3 * g + 0] = 0.0;
    m->geom_friction[3 * g + 1] = 0.0;
    m->geom_friction[3 * g + 2] = 0.0;
  }
  return 0;
}

int eng_robot_enable_newton_euler(void* robot, int enable) {
  (void)enable;
  if (robot != NULL) {
    mj_set_err(((MjRobot*)robot)->last_error, sizeof(((MjRobot*)robot)->last_error),
               "mj_shim: eng_robot_enable_newton_euler has no MuJoCo counterpart");
  }
  return -1;
}

// The Coulomb coefficient maps exactly (MuJoCo reads it off each geom); penalty and damping do NOT — the
// replaced engine scaled its penalty by contact area [Pa/m] while MuJoCo uses a solver time constant and a
// damping ratio, so porting a scenario means retuning contact softness through the model's solref.
// eng_robot_set_contact_report states the report the caller wants (1 = penalty at `stiffness` N/m, 0 = the
// solver's own table); unset, the accessors ask the environment for MJ_CONTACT_REPORT/STIFFNESS.
int eng_robot_set_contact_report(void* robot, int penalty, double stiffness) {
  if (robot == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  r->report_set = 1;
  r->report_penalty = penalty != 0;
  r->report_stiffness = stiffness;
  return 0;
}

int eng_robot_set_contact_params(void* robot, double penalty, double damping, double coulomb) {
  (void)penalty;
  (void)damping;
  if (robot == NULL) {
    return -1;
  }
  MjRobot* r = (MjRobot*)robot;
  mjModel* m = r->scene->model;
  if (coulomb >= 0.0) {
    for (int g = 0; g < m->ngeom; ++g) {
      m->geom_friction[3 * g + 0] = coulomb;
      m->geom_friction[3 * g + 1] = coulomb;
      m->geom_friction[3 * g + 2] = coulomb;
    }
  }
  return 0;
}

