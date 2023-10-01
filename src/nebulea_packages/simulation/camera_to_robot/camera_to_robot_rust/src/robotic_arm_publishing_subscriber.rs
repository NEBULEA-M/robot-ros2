/*
    Receive (i.e. subscribe) coordinates of an object in the camera reference frame, and
    publish those coordinates in the robotic arm base frame.
    -------
    Publish the coordinates of the centroid of an object to topic:
      /pos_in_robot_base_frame – The x and y position of the center of an object in centimeter coordinates
 */
use std::sync::{Arc, Mutex};
use std::time::Duration;

use nalgebra as na;
use rclrs;

use crate::coordinate_transform::CoordinateConversion;

mod coordinate_transform;

struct PublishingSubscriber {
    node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<std_msgs::msg::Float64MultiArray>>,
    publisher: Arc<rclrs::Publisher<std_msgs::msg::Float64MultiArray>>,
    cam_to_robo: coordinate_transform::CameraToRobotBaseConversion,
    data: Arc<Mutex<Option<std_msgs::msg::Float64MultiArray>>>,
}

impl PublishingSubscriber {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::create_node(&context, "publishing_subscriber")?;

        let data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&data);


        // The node subscribes to messages of type std_msgs/Float64MultiArray, over a topic named:
        // /pos_in_cam_frame
        // The callback function is called as soon as a message is received.
        // The maximum number of queued messages is 10.
         let _subscription = {
            node.create_subscription(
                "/pos_in_cam_frame",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: std_msgs::msg::Float64MultiArray| {
                    // This subscription now owns the data_cb variable
                    *data_cb.lock().unwrap() = Some(msg);
                },
            )?
        };

        // This node publishes the position in robot frame coordinates.
        // Maximum queue size of 10.
        let publisher_pos_robot_frame = node.create_publisher::<std_msgs::msg::Float64MultiArray>(
            "/pos_in_robot_base_frame",
            rclrs::QOS_PROFILE_DEFAULT,
        )?;

        // Define the displacement from frame base frame of robot to camera frame in centimeters
        let x_disp = -17.8;
        let y_disp = 24.4;
        let z_disp = 0.0;
        let rot_angle = 180.0; // angle between axes in degrees

        // Create a CameraToRobotBaseConversion object
        let cam_to_robot = coordinate_transform::CameraToRobotBaseConversion {
            angle: rot_angle,
            x: x_disp,
            y: y_disp,
            z: z_disp,
        };

        Ok(Self {
            node,
            _subscription,
            publisher: publisher_pos_robot_frame,
            cam_to_robo: cam_to_robot,
            data,
        })
    }

    fn pos_received(&self, msg: &std_msgs::msg::Float64MultiArray) {
        let object_position = msg.data.clone();

        // Coordinates of the object in the camera reference frame in centimeters
        let cam_ref_coord = na::Vector4::new(
            object_position[0],
            object_position[1],
            0.0,
            1.0,
        );

        let robot_base_frame_coord = self.cam_to_robo.convert(cam_ref_coord);

        // Capture the object's desired position (x, y)
        let object_position = vec![
            robot_base_frame_coord[0],
            robot_base_frame_coord[1],
        ];

        // Publish the coordinates to the topics
        self.publish_position(&object_position);
    }

    fn publish_position(&self, object_position: &Vec<f64>) {
        let msg = std_msgs::msg::Float64MultiArray {
            data: object_position.to_vec(), // Clone individual elements
            ..Default::default()
        };
        self.publisher.publish(&msg).unwrap();
    }

    fn republish(&self) -> Result<(), rclrs::RclrsError> {
        if let Some(s) = &*self.data.lock().unwrap() {
            println!("Receiving data {:?}", s);
            self.pos_received(&s);
        }
        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let publishing_subscriber = Arc::new(PublishingSubscriber::new(&context)?);
    let republisher_other_thread = Arc::clone(&publishing_subscriber);

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            std::thread::sleep(Duration::from_millis(3000));
            republisher_other_thread.republish()?;
        }
    });

    // Spin the node so the callback function is called.
    // Pull messages from any topics this node is subscribed to.
    // Publish any pending messages to the topics.
    rclrs::spin(Arc::clone(&publishing_subscriber.node)).map_err(|err| err.into())
}
