/*
Convert camera coordinates to robot base frame coordinates
 */

use nalgebra as na;

use rand::Rng;

pub trait CoordinateConversion {
    fn convert(&self, frame_coordinates: na::Vector4<f64>) -> na::Vector4<f64>;
}

pub struct CameraToRobotBaseConversion {
    pub angle: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl CoordinateConversion for CameraToRobotBaseConversion {
    /*
     Convert camera coordinates to robot base frame coordinates
     */
    fn convert(&self, frame_coordinates: na::Vector4<f64>) -> na::Vector4<f64> {
        // Define the rotation matrix from the robotic base frame (frame 0)
        // to the camera frame (frame c).
        let rot_mat_0_c = na::Matrix3::new(
            1.0,
            0.0,
            0.0,
            0.0,
            self.angle.cos(),
            -self.angle.sin(),
            0.0,
            self.angle.sin(),
            self.angle.cos(),
        );

        // Define the displacement vector from frame 0 to frame c
        let disp_vec_0_c = na::Vector3::new(self.x, self.y, self.z);


        let homgen_0_c = na::Matrix4::new(
            rot_mat_0_c.m11, rot_mat_0_c.m12, rot_mat_0_c.m13, disp_vec_0_c.x,
            rot_mat_0_c.m21, rot_mat_0_c.m22, rot_mat_0_c.m23, disp_vec_0_c.y,
            rot_mat_0_c.m31, rot_mat_0_c.m32, rot_mat_0_c.m33, disp_vec_0_c.z,
            0.0, 0.0, 0.0, 1.0, // Row vector for bottom of homogeneous transformation matrix
        );

        // Coordinates of the object in base reference frame

        let new_frame_coordinates = homgen_0_c * frame_coordinates;
        new_frame_coordinates
    }
}

/*
  This code is used to test the methods implemented above.
 */
fn main() {
    let x_disp = -17.8;
    let y_disp = 24.4;
    let z_disp = 0.0;
    let rot_angle = 180.0_f64.to_radians();

    let cam_to_robo = CameraToRobotBaseConversion {
        angle: rot_angle,
        x: x_disp,
        y: y_disp,
        z: z_disp,
    };

    let cm_to_pixel = 36.0 / 640.0;

    println!("Detecting an object for 3 seconds");
    let mut t = 0.0;
    let dt = 0.1;
    while t < 3.0 {
        t += dt;

        let mut rng = rand::thread_rng();
        let x = rng.gen_range(250..=450);
        let y = rng.gen_range(250..=450);

        let x_cm = x as f64 * cm_to_pixel;
        let y_cm = y as f64 * cm_to_pixel;

        let cam_ref_coord = na::Vector4::new(x_cm, y_cm, 0.0, 1.0);
        let robot_base_frame_coord = cam_to_robo.convert(cam_ref_coord);

        println!("{}: x: {}, y: {}", t, robot_base_frame_coord.x, robot_base_frame_coord.y);
    }
}
