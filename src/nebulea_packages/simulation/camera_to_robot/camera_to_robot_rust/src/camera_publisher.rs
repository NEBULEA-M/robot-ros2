/*
    Detect an object in a video stream and publish its coordinates
    from the perspective of the camera (i.e. camera reference frame)
    Note: You don't need a camera to run this node. This node just demonstrates how to create
    a publisher node in ROS2 (i.e. how to publish data to a topic in ROS2).
    -------
    Publish the coordinates of the centroid of an object to a topic:
    /pos_in_cam_frame – The position of the center of an object in centimeter coordinates
 */

use std::sync::{Arc, Mutex};
use std::time::Duration;

use rand::Rng;

struct CameraPublisherNode {
    node: Arc<rclrs::Node>,
    publisher: Arc<rclrs::Publisher<std_msgs::msg::Float64MultiArray>>,
    i_: Arc<Mutex<usize>>,
    cm_to_pixel: f64,

}

impl CameraPublisherNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::create_node(&context, "camera_publisher")?;

        // This node publishes the position of an object every 3 seconds.
        // Maximum queue size of 10.
        let publisher = node.create_publisher::<std_msgs::msg::Float64MultiArray>(
            "/pos_in_cam_frame",
            rclrs::QOS_PROFILE_DEFAULT,
        )?;

        // Centimeter to pixel conversion factor
        // Assume we measure 36.0 cm across the width of the field of view of the camera.
        // Assume the camera is 640 pixels in width and 480 pixels in height
        let cm_to_pixel = 36.0 / 640.0;

        Ok(Self {
            node,
            publisher,
            i_: Arc::new(Mutex::new(0)),
            cm_to_pixel,
        })
    }

    fn get_coordinates_of_object(&self) -> Result<(), rclrs::RclrsError> {
        let (x, y) = generate_random_coordinates();
        let (x_cm, y_cm) = convert_to_centimeters(x, y, self.cm_to_pixel);

        // Store the position of the object in a vector
        let object_position = vec![x_cm, y_cm];

        self.publish_coordinates(&object_position);
        Ok(())
    }

    fn publish_coordinates(&self, position: &Vec<f64>) {
        let i = {
            let mut guard = self.i_.lock().unwrap(); // Lock the Mutex and get a mutable reference
            *guard += 1; // Modify the value inside the Mutex
            *guard // Get the value and drop the lock
        };

        let msg = std_msgs::msg::Float64MultiArray {
            data: position.to_vec(), // Clone individual elements
            ..Default::default()
        };

        // Publish the coordinates to the topic
        println!("Publishing coordinates {}: [{:?}]", i, msg.data);

        self.publisher.publish(&msg).unwrap()
    }
}

fn generate_random_coordinates() -> (i32, i32) {
    // Center of the bounding box that encloses the detected object.
    // This is in pixel coordinates.
    // Since we don't have an actual camera and an object to detect,
    // we generate random pixel locations.
    // Assume x (width) can go from 0 to 640 pixels, and y (height) can go from 0 to 480 pixels
    let mut rng = rand::thread_rng();

    let x = rng.gen_range(250..450);
    let y = rng.gen_range(250..450);
    (x, y)
}

fn convert_to_centimeters(x: i32, y: i32, cm_to_pixel: f64) -> (f64, f64) {
    // Calculate the center of the object in centimeter coordinates
    // instead of pixel coordinates
    let x_cm = x as f64 * cm_to_pixel;
    let y_cm = y as f64 * cm_to_pixel;
    (x_cm, y_cm)
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let camera_publisher = Arc::new(CameraPublisherNode::new(&context)?);
    let camera_publisher_other_thread = Arc::clone(&camera_publisher);

    while context.ok() {
        std::thread::sleep(Duration::from_millis(3000));
        camera_publisher_other_thread.get_coordinates_of_object()?;
    }

    rclrs::spin(Arc::clone(&camera_publisher.node)).map_err(|err| err.into())
}