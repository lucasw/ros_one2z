//! load mcaps with an Odometry topic and visualize in rerun.io

use std::{env, fs, path::PathBuf};

use anyhow::{Context, Result};
use camino::Utf8Path;
use memmap::Mmap;

use rerun::external::glam;

use roslibrust_codegen_macro::find_and_generate_ros_messages;

find_and_generate_ros_messages!();

/*
fn print_type_of<T>(_: &T) -> String {
    format!("{}", std::any::type_name::<T>())
}
*/

fn map_mcap<P: AsRef<Utf8Path>>(p: P) -> Result<Mmap> {
    let fd = fs::File::open(p.as_ref()).context("Couldn't open MCAP file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

// TODO(lucasw) https://github.com/Carter12s/roslibrust/issues/158#issuecomment-2187839437
fn get_message_data_with_header<'a>(raw_message_data: std::borrow::Cow<'a, [u8]>) -> Vec<u8> {
    let len_header = raw_message_data.len() as u32;
    let mut msg_with_header = Vec::from(len_header.to_le_bytes());
    let mut message_data = Vec::from(raw_message_data);
    msg_with_header.append(&mut message_data);
    msg_with_header
}

fn ros_to_rerun_time(ros_stamp: roslibrust_codegen::Time) -> f64 {
    ros_stamp.secs as f64 + ros_stamp.nsecs as f64 / 1e9
}

fn mcap_to_rerun(rec: &rerun::RecordingStream, path: &PathBuf,
                 odom_topic: &String,
                 image_topic: &String,
                 ind: u32)
        -> Result<(), Box<dyn std::error::Error>> {
    dbg!(path);

    let mapped = map_mcap(path.display().to_string())?;

    let mut points = Vec::new();
    let mut count = 0;
    let mut image_count = 0;
    let mut timestamp = 0.0 as f64;

    for message_raw in mcap::MessageStream::new(&mapped)? {
        // println!("{:?}", print_type_of(&message));
        match message_raw {
            Ok(message) => {
                if message.channel.topic == *image_topic {
                    let msg_with_header = get_message_data_with_header(message.data);
                    match serde_rosmsg::from_slice::<sensor_msgs::CompressedImage>(&msg_with_header) {
                        Ok(image_msg) => {
                            if image_count % 120 == 0 {
                                rec.set_time_seconds("sensor_time", ros_to_rerun_time(image_msg.header.stamp));
                                let img = rerun::EncodedImage::from_file_contents(image_msg.data);
                                // rec.log(message.channel.topic.clone(), &img)?;
                                rec.log("image", &img)?;
                            }
                            image_count += 1;
                        },
                        Err(e) => {
                            println!("{:?}", e);
                        },
                    }

                } else if message.channel.topic == *odom_topic {
                    // message.channel.schema == "nav_msgs/Odometry"
                    // println!("{:?}", message.channel.schema);

                    // https://github.com/adnanademovic/serde_rosmsg/blob/master/src/lib.rs#L9

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

                    let msg_with_header = get_message_data_with_header(message.data);

                    match serde_rosmsg::from_slice::<nav_msgs::Odometry>(&msg_with_header) {
                        Ok(odom_msg) => {
                            // println!("{:#?}", odom_msg);
                            let pos = &odom_msg.pose.pose.position;
                            timestamp = ros_to_rerun_time(odom_msg.header.stamp);
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
        }  // message_raw
    }

    println!("{} -> {} points extracted", count, points.len());

    let mut points_vec = Vec::new();
    points_vec.push(points);

    rec.set_time_seconds("sensor_time", timestamp);
    /*
    // this will move with the timeline
    rec.log(
        format!("{odom_topic}/position"),
        &rerun::LineStrips3D::new(points_vec.clone())
            .with_colors([rerun::Color::from([128, 128, 128, 255])]),
    )?;
    */

    // this will be persistent
    let filename = path.file_stem().unwrap().to_string_lossy();
    rec.log(
        format!("{odom_topic}/position/{filename}"),
        &rerun::LineStrips3D::new(points_vec)
            .with_colors([rerun::Color::from([180, ((ind * 5) % 256) as u8, (ind % 256) as u8, 255])]),
    )?;

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun::RecordingStreamBuilder::new("bag_odom_to_rerun").spawn()?;

    let args: Vec<String> = env::args().collect();

    let path = &args[1];
    let odom_topic = &args[2];
    let image_topic = &args[3];

    // dbg!(args);
    dbg!(path);
    dbg!(odom_topic);
    dbg!(image_topic);

    let mut paths: Vec<_> = std::fs::read_dir(path).unwrap()
                                                   .map(|r| r.unwrap())
                                                   .collect();
    paths.sort_by_key(|dir| dir.path());

    let mut ind = 0;
    for entry in paths {
        ind += 1;
        // let entry = entry?;
        match mcap_to_rerun(&rec, &entry.path(), odom_topic, image_topic, ind) {
            Ok(()) => {
            },
            Err(e) => {
                println!("{:?}", e);
            },
        }
    }

    Ok(())
}
