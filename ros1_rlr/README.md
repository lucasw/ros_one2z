
ros1 roslibrust native node example/s with several publishers and subscribers each

https://github.com/Carter12s/roslibrust
https://docs.rs/roslibrust/latest/roslibrust/

Supplying minimal ROS_PACKAGE_PATH works best (and probably reduces compile times):

```
ROS_PACKAGE_PATH=`rospack find std_msgs`:`rospack find geometry_msgs` cargo run
```

Could have a method to figure out dependencies from package.xml and put only those on the path.


TODO - compare similar nodes with rosrust equivalents, see how much cpu is used and what message latencies are like.
