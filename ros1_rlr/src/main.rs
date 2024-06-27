// TODO(lucasw) use lib.rs for this later, see https://docs.rs/roslibrust/latest/roslibrust/#message-generation
roslibrust_codegen_macro::find_and_generate_ros_messages!();

fn time_now() -> roslibrust_codegen::Time {
    use std::time::SystemTime;
    let duration_since_epoch = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
    let stamp = roslibrust_codegen::Time {
        secs: duration_since_epoch.as_secs() as u32,
        nsecs: (duration_since_epoch.as_nanos() % 1e9 as u128) as u32,
    };
    stamp
}

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    /*
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        // .without_timestamps() // required for running wsl2
        .init()
        .unwrap();
    */

    let mut ns = String::from("");
    let args = std::env::args();
    {
        // get namespace
        for arg in args {
            if arg.starts_with("__ns:=") {
               ns = arg.replace("__ns:=", "");
            }
        }
    }

    // TODO(lucasw) support remapping

    // TODO(lucasw) need to have leading slash on node name and topic to fully work,
    // so figure out namespace then prefix it to all of these
    let full_node_name = &format!("/{ns}/example").replace("//", "/");
    println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await.unwrap();
    let point_pub = nh.advertise::<geometry_msgs::PointStamped>(&format!("{ns}/point"), 2)
        .await.unwrap();

    let mut seq = 0;

    loop {
        let mut point_msg = geometry_msgs::PointStamped::default();
        point_msg.header.stamp = time_now();
        point_msg.header.seq = seq;
        let pub_rv = point_pub.publish(&point_msg).await;
        match pub_rv {
            Ok(()) => {
                // println!("published command");
            },
            Err(e) => {
                eprintln!("command pub error, exiting {e}");
                break;
            }
        }
        if !nh.is_ok() {
            println!("node handle not ok, exiting");
            break;
        }
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        seq += 1;
    }

    Ok(())
}
