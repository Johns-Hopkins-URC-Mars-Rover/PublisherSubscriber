# Publisher and Subscriber

This is the initial attempt to send and receive data.
The publisher publishes data through a topic that a subscriber may subscribe to in order to recieve the data.


## Features

- Send data with Ros2
- Receive data with Ros2
- Could deal with different types of data by customizing function


## Ros2 Workspace Setup
Add to `package.xml`:
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Add to entry points in `setup.py`:
```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_file:main',
                'listener = py_pubsub.subscriber_file:main',
        ],
},
```


## Build/Run
1. `colcon build --packages-select [package]`

2. `source install/setup.bash`

3. `ros2 run [package] [talker/listener]`


## Publisher Usage:

```python
rclpy.init(args=args)

publisher = Publisher('sample_publisher', 'topic', example_data_func)
rclpy.spin(publisher)

publisher.destroy_node() # optional
rclpy.shutdown()
```
Where `example_data_func` is the function that will return the data to be sent when called.


## Subscriber Usage:

```python
rclpy.init(args=args)

subscriber = Subscriber('sample_subscriber', 'topic', example_recieve_func)
rclpy.spin(subscriber)
 
subscriber.destroy_node() # optional
rclpy.shutdown()
```
Where `example_recieve_func` is the function that will handle the data recieved when called.
