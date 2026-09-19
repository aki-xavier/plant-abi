// ffi.rs — the eng_* C ABI, one declaration per function in cabi/eng_shim.h: the whole contract
// between the Rust control core and the physics engine. Returns are 0 on success and -1 on failure,
// with the cause in eng_robot_last_error() (also mirrored to stderr).
//
// eng_robot_attach* takes the CALLER's error buffer and its capacity, so a message is truncated to it
// rather than written past its end.

#![allow(non_camel_case_types)]

use std::os::raw::{c_char, c_double, c_float, c_int, c_uchar, c_void};

pub type Handle = *mut c_void;

extern "C" {
    pub fn eng_context_open() -> Handle;
    pub fn eng_context_close(ctx: Handle);

    pub fn eng_scene_open(ctx: Handle, name: *const c_char) -> Handle;
    pub fn eng_scene_free(ctx: Handle, scene: Handle);
    /// Adds a static rigid box whose shape is a mesh, centred at world (px, py, pz); only one wall per
    /// scene (later calls are no-ops).
    pub fn eng_scene_add_box(
        scene: Handle,
        shape_path: *const c_char,
        px: c_float,
        py: c_float,
        pz: c_float,
    ) -> c_int;
    /// Adds an infinite static ground plane (normal . x = distance).
    pub fn eng_scene_add_plane(
        scene: Handle,
        nx: c_float,
        ny: c_float,
        nz: c_float,
        distance: c_float,
    ) -> c_int;

    pub fn eng_robot_attach(
        scene: Handle,
        urdf: *const c_char,
        ee: *const c_char,
        out: *mut Handle,
        err: *mut u8,
        err_cap: c_int,
    ) -> c_int;
    /// Keeps the injected world joint Free: the base owns 6 leading DOFs.
    pub fn eng_robot_attach_floating(
        scene: Handle,
        urdf: *const c_char,
        ee: *const c_char,
        out: *mut Handle,
        err: *mut u8,
        err_cap: c_int,
    ) -> c_int;
    /// weld_root (0 = floating base), voxel_meters (ignored — the current engine collides meshes
    /// directly, so pass 0).
    pub fn eng_robot_attach_ex(
        scene: Handle,
        urdf: *const c_char,
        ee: *const c_char,
        weld_root: c_int,
        voxel_meters: c_double,
        out: *mut Handle,
        err: *mut u8,
        err_cap: c_int,
    ) -> c_int;
    pub fn eng_robot_free(robot: Handle);

    pub fn eng_robot_num_dofs(robot: Handle) -> c_int;
    pub fn eng_robot_set_state(robot: Handle, q: *const c_float, v: *const c_float) -> c_int;
    /// Advances n_sub times by dt (the simulated time is n_sub * dt).
    pub fn eng_robot_step(robot: Handle, tau: *const c_float, n_sub: c_int, dt: c_double) -> c_int;
    pub fn eng_robot_get_q(robot: Handle, q_out: *mut c_float) -> c_int;
    pub fn eng_robot_get_v(robot: Handle, v_out: *mut c_float) -> c_int;
    pub fn eng_robot_ee_pos(robot: Handle, pos_out: *mut c_float) -> c_int;
    /// No MuJoCo counterpart: fails loudly instead of pretending.
    pub fn eng_robot_enable_newton_euler(robot: Handle, enable: c_int) -> c_int;
    pub fn eng_robot_keep_viscous_only(robot: Handle) -> c_int;

    /// Retunes every robot-link/static pair: penalty [Pa/m], normal damping, Coulomb friction (0 leaves
    /// a field untouched).
    pub fn eng_robot_set_contact_params(
        robot: Handle,
        penalty: c_double,
        damping: c_double,
        coulomb: c_double,
    ) -> c_int;
    /// States the contact REPORT the caller wants: 1 = the penalty report at `stiffness` N/m, 0 = the
    /// solver's own per-contact table; before any call the shim asks the environment
    /// (`MJ_CONTACT_REPORT` / `MJ_CONTACT_STIFFNESS`).
    pub fn eng_robot_set_contact_report(
        robot: Handle,
        penalty: c_int,
        stiffness: c_double,
    ) -> c_int;
    /// The solver's own applied force and torque, not the sum of the per-link tables under the penalty
    /// report.
    pub fn eng_robot_contact_force(robot: Handle, out_force: *mut c_float) -> c_int;
    pub fn eng_robot_contact_torque(robot: Handle, out_torque: *mut c_float) -> c_int;
    /// Both tables use the CALLER's slot layout (slot s = link s+1 in chain order, the LAST slot =
    /// floating root), return a LINK count and truncate a shorter buffer silently; they are LOW-PASSED
    /// and stateful, so read them once per tick.
    pub fn eng_robot_link_contact_forces(
        robot: Handle,
        out: *mut c_float,
        n_links_max: c_int,
    ) -> c_int;
    pub fn eng_robot_link_contact_wrenches(
        robot: Handle,
        out: *mut c_float,
        n_links_max: c_int,
    ) -> c_int;

    pub fn eng_robot_last_error(robot: Handle) -> *const c_char;

    /// Draws one frame, packed RGB8 (width * height * 3 bytes, top row first); a null lookat follows the
    /// robot's centre of mass.
    pub fn eng_scene_render(
        scene: Handle,
        width: c_int,
        height: c_int,
        lookat: *const c_float,
        distance: c_float,
        azimuth: c_float,
        elevation: c_float,
        rgb: *mut c_uchar,
        rgb_cap: c_int,
    ) -> c_int;
    pub fn eng_scene_render_error(scene: Handle) -> *const c_char;
}
