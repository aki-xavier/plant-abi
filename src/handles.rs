// handles.rs — the engine handles, owned in Rust: the C ABI hands out raw voidptr fields
// (context, scene, robot) and frees them in close(), so each wrapper owns its own handle. The ABI's
// order is enforced by the lifetimes — a Robot cannot outlive its Scene, a Scene cannot outlive its
// Engine. This is the engine's own side of the boundary; the note at the end of the file says why the
// Plant contract the controllers consume is NOT here.

use std::ffi::CString;
use std::marker::PhantomData;

use crate::ffi::Handle;

pub const ERR_BUF: usize = 512;

/// Engine owns the process-wide engine context (VFS and error state).
pub struct Engine {
    ctx: Handle,
}

impl Engine {
    pub fn open() -> Result<Self, String> {
        let ctx = unsafe { crate::ffi::eng_context_open() };
        if ctx.is_null() {
            return Err("eng_context_open returned null".to_string());
        }
        Ok(Engine { ctx })
    }

    pub fn scene(&self, name: &str) -> Result<Scene<'_>, String> {
        let c = CString::new(name).map_err(|_| "scene name contains a NUL".to_string())?;
        let scene = unsafe { crate::ffi::eng_scene_open(self.ctx, c.as_ptr()) };
        if scene.is_null() {
            return Err("eng_scene_open returned null".to_string());
        }
        Ok(Scene {
            ctx: self.ctx,
            scene,
            _e: PhantomData,
        })
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        unsafe { crate::ffi::eng_context_close(self.ctx) };
    }
}

pub struct Scene<'e> {
    ctx: Handle,
    scene: Handle,
    _e: PhantomData<&'e Engine>,
}

impl Scene<'_> {
    /// attach_floating keeps the injected world joint Free (a floating base).
    pub fn attach_floating(&self, urdf: &str, ee: &str) -> Result<Robot<'_>, String> {
        let urdf_c = CString::new(urdf).map_err(|_| "urdf path contains a NUL".to_string())?;
        let ee_c = CString::new(ee).map_err(|_| "ee link contains a NUL".to_string())?;
        let mut out: Handle = std::ptr::null_mut();
        let mut err = vec![0u8; ERR_BUF];
        let rc = unsafe {
            crate::ffi::eng_robot_attach_floating(
                self.scene,
                urdf_c.as_ptr(),
                ee_c.as_ptr(),
                &mut out,
                err.as_mut_ptr(),
                ERR_BUF as i32,
            )
        };
        if rc != 0 {
            return Err(format!("attach failed: {}", buf_message(&err)));
        }
        Ok(Robot {
            robot: out,
            _s: PhantomData,
        })
    }

    pub fn add_plane(&self, normal: [f32; 3], distance: f32) -> Result<(), String> {
        let rc = unsafe {
            crate::ffi::eng_scene_add_plane(self.scene, normal[0], normal[1], normal[2], distance)
        };
        if rc != 0 {
            return Err("eng_scene_add_plane failed".to_string());
        }
        Ok(())
    }
}

impl Drop for Scene<'_> {
    fn drop(&mut self) {
        unsafe { crate::ffi::eng_scene_free(self.ctx, self.scene) };
    }
}

pub struct Robot<'s> {
    robot: Handle,
    _s: PhantomData<&'s Scene<'s>>,
}

impl Robot<'_> {
    pub fn num_dofs(&self) -> i32 {
        unsafe { crate::ffi::eng_robot_num_dofs(self.robot) }
    }

    pub fn set_state(&self, q: &[f32], v: &[f32]) -> Result<(), String> {
        let rc = unsafe { crate::ffi::eng_robot_set_state(self.robot, q.as_ptr(), v.as_ptr()) };
        if rc != 0 {
            return Err(format!("set_state failed: {}", self.last_error()));
        }
        Ok(())
    }

    /// step advances the engine n_sub times by dt (n_sub * dt of simulated time). dt == 0 is the
    /// contract's query-refresh step: the world does not advance, but the contact readout is brought
    /// up to date with the pose.
    pub fn step(&self, tau: &[f32], n_sub: i32, dt: f64) -> Result<(), String> {
        let rc = unsafe { crate::ffi::eng_robot_step(self.robot, tau.as_ptr(), n_sub, dt) };
        if rc != 0 {
            return Err(format!("step failed: {}", self.last_error()));
        }
        Ok(())
    }

    pub fn get_q(&self, q_out: &mut [f32]) -> Result<(), String> {
        let rc = unsafe { crate::ffi::eng_robot_get_q(self.robot, q_out.as_mut_ptr()) };
        if rc != 0 {
            return Err(format!("get_q failed: {}", self.last_error()));
        }
        Ok(())
    }

    pub fn ee_pos(&self) -> Result<[f32; 3], String> {
        let mut p = [0.0f32; 3];
        let rc = unsafe { crate::ffi::eng_robot_ee_pos(self.robot, p.as_mut_ptr()) };
        if rc != 0 {
            return Err(format!("ee_pos failed: {}", self.last_error()));
        }
        Ok(p)
    }

    pub fn keep_viscous_only(&self) -> Result<(), String> {
        let rc = unsafe { crate::ffi::eng_robot_keep_viscous_only(self.robot) };
        if rc != 0 {
            return Err("keep_viscous_only failed".to_string());
        }
        Ok(())
    }

    pub fn set_contact_params(
        &self,
        penalty: f64,
        damping: f64,
        coulomb: f64,
    ) -> Result<(), String> {
        let rc = unsafe {
            crate::ffi::eng_robot_set_contact_params(self.robot, penalty, damping, coulomb)
        };
        if rc != 0 {
            return Err("set_contact_params failed".to_string());
        }
        Ok(())
    }

    /// link_contact_forces reads the per-link world-frame forces into the CALLER's slot layout
    /// (slot s = link s+1 in chain order, last slot = floating root); a short buffer truncates
    /// silently. Stateful low-pass, so read once per tick; an error leaves the buffer untouched.
    pub fn link_contact_forces(&self, n_slots: usize) -> Result<Vec<f32>, String> {
        let mut out = vec![0.0f32; 3 * n_slots];
        let rc = unsafe {
            crate::ffi::eng_robot_link_contact_forces(self.robot, out.as_mut_ptr(), n_slots as i32)
        };
        if rc < 0 {
            return Err(format!("link_contact_forces failed: {}", self.last_error()));
        }
        Ok(out)
    }

    /// the same table with the moment about each link's own origin: [fx fy fz tx ty tz] per slot, so
    /// cop_x = -ty/fz and cop_y = +tx/fz is that link's pressure point.
    pub fn link_contact_wrenches(&self, n_slots: usize) -> Result<Vec<f32>, String> {
        let mut out = vec![0.0f32; 6 * n_slots];
        let rc = unsafe {
            crate::ffi::eng_robot_link_contact_wrenches(
                self.robot,
                out.as_mut_ptr(),
                n_slots as i32,
            )
        };
        if rc < 0 {
            return Err(format!(
                "link_contact_wrenches failed: {}",
                self.last_error()
            ));
        }
        Ok(out)
    }

    /// contact_force is the aggregate force over the robot's own bodies as the engine's solver applied
    /// it — not the sum of the per-link tables under the penalty readout (eng_shim.h).
    pub fn contact_force(&self) -> Result<[f32; 3], String> {
        let mut f = [0.0f32; 3];
        let rc = unsafe { crate::ffi::eng_robot_contact_force(self.robot, f.as_mut_ptr()) };
        if rc < 0 {
            return Err(format!("contact_force failed: {}", self.last_error()));
        }
        Ok(f)
    }

    pub fn last_error(&self) -> String {
        unsafe { cstr_message(crate::ffi::eng_robot_last_error(self.robot)) }
    }
}

impl Drop for Robot<'_> {
    fn drop(&mut self) {
        // the storage belongs to the scene; the release call is still made, in order
        unsafe { crate::ffi::eng_robot_free(self.robot) };
    }
}

/// buf_message reads a NUL-terminated message out of a fixed error buffer (the attach funnel's
/// `err`/`err_cap` contract).
pub fn buf_message(buf: &[u8]) -> String {
    let end = buf.iter().position(|b| *b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..end]).into_owned()
}

unsafe fn cstr_message(p: *const std::os::raw::c_char) -> String {
    if p.is_null() {
        return String::new();
    }
    std::ffi::CStr::from_ptr(p).to_string_lossy().to_string()
}

// ---- who owns what ---------------------------------------------------------
// The Plant contract the controllers consume is the sibling crate ../control-base's
// (control_base::plant::Plant). It left because a contract and an engine cannot share a file: while
// the trait sat at the bottom of a plant module, a control crate could not name it without also naming
// the handles above. What is stated HERE is the engine's own side — the handles, and the C ABI they
// wrap (ffi.rs) — which is this crate's whole job.
//
// The plants that USE these handles are their products', not this crate's: simu's CEnginePlant drives
// an arm, its BipedPlant mirrors a walking machine, and this crate learns about neither. Anything
// above the ABI that more than one of them needs belongs in ../control-base, where the contract lives.
