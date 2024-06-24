//! load mcaps with an Odometry topic and visualize in rerun.io

use std::{env, fs};

use anyhow::{Context, Result};
use camino::Utf8Path;
use memmap::Mmap;

use rerun::external::glam;

use roslibrust_codegen_macro::find_and_generate_ros_messages;

find_and_generate_ros_messages!();

fn print_type_of<T>(_: &T) -> String {
    format!("{}", std::any::type_name::<T>())
}

fn map_mcap<P: AsRef<Utf8Path>>(p: P) -> Result<Mmap> {
    let fd = fs::File::open(p.as_ref()).context("Couldn't open MCAP file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun::RecordingStreamBuilder::new("bag_odom_to_rerun").spawn()?;

    let args: Vec<String> = env::args().collect();

    let path = &args[1];
    let odom_topic = &args[2];

    // dbg!(args);
    dbg!(path);
    dbg!(odom_topic);

    let mapped = map_mcap(path)?;

    let mut points = Vec::new();
    let mut count = 0;

    for message_raw in mcap::MessageStream::new(&mapped)? {
        // println!("{:?}", print_type_of(&message));
        match message_raw {
            Ok(message) => {
                if message.channel.topic == *odom_topic {  // && message.channel.schema == "nav_msgs/Odometry" {
                    // println!("{:?}", message.channel.schema);

                    // https://github.com/adnanademovic/serde_rosmsg/blob/master/src/lib.rs#L9
                    let len_header = message.data.len() as u32;
                    let mut msg_with_header = Vec::from(len_header.to_le_bytes());
                    let mut message_data = Vec::from(message.data);
                    msg_with_header.append(&mut message_data);

                    /*
                    {
                        let mut test_msg = marti_common_msgs::Float32Stamped::default();
                        test_msg.header.seq = 3;
                        test_msg.header.frame_id = "test".to_string();
                        test_msg.header.stamp.secs = 1;
                        test_msg.header.stamp.nsecs = 7;
                        test_msg.value = 1.0;
                        let test_data = serde_rosmsg::to_vec(&test_msg).unwrap();
                        println!("test {} - {:02x?}", test_data.len(), &test_data[..]);
                        println!("mcap {} - {:02x?}", msg_with_header_vec.len(), &msg_with_header_vec[..]);
                    }
                    match serde_rosmsg::from_slice::<marti_common_msgs::Float32Stamped>(&msg_with_header) {
                    */

                    match serde_rosmsg::from_slice::<nav_msgs::Odometry>(&msg_with_header) {
                        Ok(odom_msg) => {
                            // println!("{:#?}", odom_msg);
                            let pos = &odom_msg.pose.pose.position;
                            // TODO(lucasw) get min and max of xyz
                            if count % 10 == 0 {
                                let point = glam::vec3(pos.x as f32, pos.y as f32, pos.z as f32);
                                points.push(point);
                            }

                            count += 1;
                        },
                        Err(e) => {
                            println!("{:?}", e);
                        },
                    }
                }
            }
            Err(e) => {
                println!("{:?}", e);
            },
        }
    }

    println!("{} -> {} points extracted", count, points.len());

    let mut points_vec = Vec::new();
    points_vec.push(points);

    rec.log(
        format!("{odom_topic}/position"),
        &rerun::LineStrips3D::new(points_vec)
            .with_colors([rerun::Color::from([128, 128, 128, 255])]),
    )?;

    Ok(())
}
