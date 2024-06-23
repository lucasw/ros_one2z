//! load mcaps with an Odometry topic and visualize in rerun.io

use std::{env, fs};

use anyhow::{Context, Result};
use camino::Utf8Path;
use memmap::Mmap;

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

    for message_raw in mcap::MessageStream::new(&mapped)? {
        // println!("{:?}", print_type_of(&message));
        match message_raw {
            Ok(message) => {
                if message.channel.topic == *odom_topic {  // && message.channel.schema == "nav_msgs/Odometry" {
                    println!("{:?}", message.channel.schema);
                    let ros_msg = nav_msgs::Odometry::default();
                    println!("{:#?}", ros_msg);
                    // TODO(lucasw) do something with serde to deserialize/decode message.data

                    let ts = message.publish_time;
                    println!(
                        "{} {} [{}] [{}]...",
                        ts,
                        message.channel.topic,
                        message
                            .channel
                            .schema
                            .as_ref()
                            .map(|s| s.name.as_str())
                            .unwrap_or_default(),
                        message
                            .data
                            .iter()
                            .take(10)
                            .map(|b| b.to_string())
                            .collect::<Vec<_>>()
                            .join(" ")
                    );
                }
            }
            Err(e) => {
                println!("{:?}", e);
            },
        }
    }
    Ok(())
}
