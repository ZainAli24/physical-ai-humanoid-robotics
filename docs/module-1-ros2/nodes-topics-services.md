---
sidebar_position: 2
---

# Nodes, Topics, and Services

## Prerequisites
- Completed ROS 2 Fundamentals chapter
- ROS 2 Humble installed and configured
- Basic Python 3.10+ programming knowledge
- Understanding of object-oriented programming concepts

## Introduction

In the previous chapter, we learned about the ROS 2 computational graph—a network of processes working together to control a robot. Now we'll dive into the fundamental building blocks that make this distributed communication possible: **nodes**, **topics**, and **services**.

Modern robotic systems are inherently complex, requiring multiple subsystems to work simultaneously: sensors collecting data, processors making decisions, and actuators executing commands. ROS 2's communication paradigm elegantly solves this challenge by allowing independent processes to exchange information through well-defined patterns. Understanding these patterns is essential for building scalable, maintainable robot applications.

In this chapter, you'll learn when to use publish-subscribe communication (topics) versus request-response patterns (services), how to structure your application into logical nodes, and most importantly, you'll write real Python code that demonstrates these concepts in action. By the end, you'll have hands-on experience building multi-node systems that mirror real-world robotics architectures.

## Understanding Nodes

### What Are Nodes?

A **node** is an executable process that performs a specific computational task in your robot system. Think of nodes as specialized workers in a factory: each has a distinct responsibility, and together they accomplish complex operations. In ROS 2, every node is built on the \`rclpy.node.Node\` class, which provides the infrastructure for communication, parameter management, and lifecycle control.

### When to Create Multiple Nodes

The decision to split functionality into separate nodes versus keeping it in one node is architectural. Here are key principles:

**Create separate nodes when:**
- **Different update rates are needed**: A camera might publish at 30 Hz while a lidar publishes at 10 Hz
- **Independent failure domains**: If one component crashes, others continue running
- **Different languages or dependencies**: You can mix Python, C++, and other ROS 2 client libraries
- **Logical separation**: Perception, planning, and control are distinct concerns
- **Reusability**: A generic laser scanner node can be reused across multiple robots

**Keep functionality in one node when:**
- **Tight coupling exists**: Components share significant state or need synchronous execution
- **Performance is critical**: Inter-process communication has overhead compared to function calls
- **Simplicity matters**: For small projects, one well-structured node may be clearer

In ROS 2, nodes are first-class citizens. Each has a unique name in the computational graph, can be discovered dynamically, and can be launched, stopped, or reconfigured independently. This modularity is what makes ROS 2 systems maintainable as they scale from prototypes to production robots.

### Node Anatomy

Every ROS 2 Python node follows a consistent structure:
1. **Imports**: ROS 2 client library and message types
2. **Class definition**: Inherits from \`rclpy.node.Node\`
3. **Constructor**: Initializes publishers, subscribers, timers, services
4. **Callback methods**: Handle events (messages, timer ticks, service requests)
5. **Main function**: Initializes ROS, creates node instance, spins

This pattern will become second nature as you work through the examples below.

## Topics and Publish-Subscribe

### The Publish-Subscribe Pattern

**Topics** implement a many-to-many, asynchronous communication pattern. Publishers send data without knowing who (if anyone) is listening. Subscribers receive data without knowing who sent it. This decoupling is powerful for robotics where components are added, removed, or replaced frequently.

\`\`\`mermaid
sequenceDiagram
    participant Publisher
    participant Topic
    participant Subscriber1
    participant Subscriber2

    Publisher->>Topic: publish(msg)
    Topic-->>Subscriber1: msg
    Topic-->>Subscriber2: msg
    Publisher->>Topic: publish(msg)
    Topic-->>Subscriber1: msg
    Topic-->>Subscriber2: msg
\`\`\`

*Figure 1: Publish-Subscribe Pattern - One publisher sends data to multiple subscribers through a topic*

### When to Use Topics

Topics are ideal for:
- **Streaming sensor data**: Cameras, lidars, IMUs publishing continuously
- **Status updates**: Robot battery level, error messages, diagnostic information
- **Command streams**: Velocity commands for motor control
- **Logging and visualization**: Debug data that may have zero or multiple consumers

The key characteristic is **continuous data flow** where the producer doesn't need acknowledgment from consumers.

### Topic Communication Benefits

1. **Anonymous communication**: Publishers don't track subscribers
2. **Dynamic discovery**: New subscribers can join anytime
3. **Scalability**: Adding subscribers doesn't impact publishers
4. **Fault isolation**: A crashed subscriber doesn't affect the publisher
5. **Quality of Service (QoS)**: Fine-tune reliability, durability, history policies

### Message Flow in Topics

When a publisher sends a message:
1. The message is serialized (converted to bytes)
2. The DDS middleware propagates it to all matched subscribers
3. Each subscriber's middleware deserializes the message
4. The subscriber's callback function is invoked with the message

This happens asynchronously—publishers never block waiting for subscribers.

## Services and Request-Response

### The Service Pattern

**Services** implement a one-to-one, synchronous request-response pattern. A client sends a request and blocks (or waits asynchronously) until the server sends a reply. This pattern mirrors function calls across process boundaries.

\`\`\`mermaid
sequenceDiagram
    participant Client
    participant Service
    participant Server

    Client->>Service: request
    Service->>Server: request
    Server->>Server: process request
    Server->>Service: response
    Service->>Client: response
\`\`\`

*Figure 2: Service Request-Response Pattern - Client waits for server's response*

### When to Use Services vs Topics

**Use services when:**
- **Immediate response is needed**: "What is the robot's current pose?"
- **Action triggers computation**: "Calculate a path from A to B"
- **State queries**: "Is the gripper open?"
- **Configuration changes**: "Set maximum velocity to 2.0 m/s"
- **Transactional operations**: Operations that should complete or fail atomically

**Use topics when:**
- **Continuous data streams**: Sensor readings, odometry updates
- **One-way notifications**: Alerts, logs, status messages
- **Multiple consumers**: Many nodes need the same data
- **Temporal decoupling**: Producer and consumer don't need synchronized timing

A common pattern: use topics for data flow and services for control/configuration.

### Service Guarantees

ROS 2 services provide:
- **Request uniqueness**: Each request gets exactly one response
- **Timeout support**: Clients can abandon slow requests
- **Synchronous or async**: Block on call or use callbacks/futures
- **Type safety**: Request and response messages are strongly typed

However, services don't guarantee:
- **Reliability**: If the server crashes, the request fails
- **Multiple responses**: Each call returns one response (use actions for feedback)
- **Persistence**: No built-in retry or queuing

## Code Examples

Now let's build these concepts with real Python code. We'll start simple and progressively add complexity.

### Example 1: Minimal Publisher

This example demonstrates the simplest possible publisher that sends string messages at 1 Hz.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """Publishes string messages at 1 Hz."""

    def __init__(self):
        super().__init__('minimal_publisher')
        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Create timer: period in seconds, callback function
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info('MinimalPublisher initialized')

    def timer_callback(self):
        """Called every 1 second by the timer."""
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts:**
- **Line 8**: Every node inherits from `rclpy.node.Node`
- **Line 12**: `create_publisher()` takes message type, topic name, and QoS queue size
- **Line 14**: `create_timer()` schedules periodic callbacks (1.0 second interval)
- **Line 19**: Publishers send messages with `.publish()`
- **Line 20**: `get_logger()` provides ROS 2's logging system

**Expected Output:**
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

### Example 2: Minimal Subscriber

This subscriber listens to the publisher above and prints received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """Subscribes to string messages and logs them."""

    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create subscription: message type, topic name, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('MinimalSubscriber initialized')

    def listener_callback(self, msg):
        """Called automatically when a message arrives."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts:**
- **Line 12-17**: `create_subscription()` registers a callback for incoming messages
- **Line 20**: Callback receives the message object as a parameter
- **Asynchronous execution**: The callback runs whenever data arrives, not on a fixed schedule

**Expected Output:**
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 2"
...
```

**Try It:**
Run both nodes in separate terminals:
```bash
# Terminal 1
ros2 run my_package minimal_publisher

# Terminal 2
ros2 run my_package minimal_subscriber
```

### Example 3: Temperature Sensor Publisher

A more realistic example: simulating a temperature sensor with Float64 messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class TemperatureSensorPublisher(Node):
    """Simulates a temperature sensor publishing readings."""

    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Float64, 'temperature', 10)
        # Publish at 2 Hz (twice per second)
        self.timer = self.create_timer(0.5, self.publish_temperature)
        self.get_logger().info('Temperature sensor started')

    def publish_temperature(self):
        """Simulate temperature reading with noise."""
        # Base temperature 20°C with ±2°C random variation
        temp = 20.0 + random.uniform(-2.0, 2.0)

        msg = Float64()
        msg.data = round(temp, 2)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data}°C')


def main(args=None):
    rclpy.init(args=args)
    sensor = TemperatureSensorPublisher()
    rclpy.spin(sensor)
    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts:**
- **Line 3**: `Float64` is a standard ROS 2 message type for numeric data
- **Line 14**: Timer runs at 2 Hz (0.5 second period)
- **Real-world pattern**: Sensors typically publish at fixed rates matching their hardware sampling frequency

**Expected Output:**
```
[INFO] [temperature_sensor]: Temperature: 19.23°C
[INFO] [temperature_sensor]: Temperature: 21.87°C
[INFO] [temperature_sensor]: Temperature: 18.54°C
...
```

### Example 4: Temperature Monitor Subscriber

This subscriber processes temperature data and triggers warnings.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TemperatureMonitor(Node):
    """Monitors temperature and logs warnings for extreme values."""

    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float64,
            'temperature',
            self.temperature_callback,
            10
        )
        # Configure thresholds
        self.temp_min = 15.0  # °C
        self.temp_max = 25.0  # °C
        self.reading_count = 0
        self.get_logger().info('Temperature monitor started')

    def temperature_callback(self, msg):
        """Process temperature reading and check thresholds."""
        temp = msg.data
        self.reading_count += 1

        if temp < self.temp_min:
            self.get_logger().warn(
                f'LOW TEMPERATURE ALERT: {temp}°C (min: {self.temp_min}°C)'
            )
        elif temp > self.temp_max:
            self.get_logger().warn(
                f'HIGH TEMPERATURE ALERT: {temp}°C (max: {self.temp_max}°C)'
            )
        else:
            self.get_logger().info(f'Temperature OK: {temp}°C')

        # Log statistics every 10 readings
        if self.reading_count % 10 == 0:
            self.get_logger().info(
                f'Processed {self.reading_count} temperature readings'
            )


def main(args=None):
    rclpy.init(args=args)
    monitor = TemperatureMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts:**
- **Line 28-36**: Conditional logic based on sensor data
- **State tracking**: The node maintains `reading_count` across callbacks
- **Logging levels**: `warn()` for alerts, `info()` for normal operation

**Expected Output:**
```
[INFO] [temperature_monitor]: Temperature OK: 19.23°C
[WARN] [temperature_monitor]: HIGH TEMPERATURE ALERT: 26.87°C (max: 25.0°C)
[INFO] [temperature_monitor]: Temperature OK: 21.54°C
[INFO] [temperature_monitor]: Processed 10 temperature readings
...
```

### Example 5: Adding Service Server

Services handle request-response interactions. This server adds two integers.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """Service server that adds two integers."""

    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create service: service type, service name, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('AddTwoInts service ready')

    def add_two_ints_callback(self, request, response):
        """Handle service request and return response."""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts:**
- **Line 3**: `AddTwoInts` is a standard ROS 2 service type with `.a`, `.b` fields in request and `.sum` in response
- **Line 12-16**: `create_service()` registers the service with a callback
- **Line 19**: Service callbacks receive `request` and `response` objects and must return the response

**Expected Output (when client sends requests):**
```
[INFO] [add_two_ints_server]: Request: 5 + 3 = 8
[INFO] [add_two_ints_server]: Request: 10 + 20 = 30
...
```

### Example 6: Adding Service Client

This client sends requests to the AddTwoInts service asynchronously.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """Service client that requests integer addition."""

    def __init__(self):
        super().__init__('add_two_ints_client')
        # Create client: service type, service name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        """Send async request and return future."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')
        # Returns a future object
        return self.cli.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()

    # Send request
    future = client.send_request(10, 15)

    # Wait for response
    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        result = future.result()
        client.get_logger().info(f'Result: {result.sum}')
    else:
        client.get_logger().error('Service call failed')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts:**
- **Line 15**: `wait_for_service()` blocks until the server is available
- **Line 26**: `call_async()` sends the request without blocking
- **Line 37**: `spin_until_future_complete()` waits for the response

**Expected Output:**
```
[INFO] [add_two_ints_client]: Sending request: 10 + 15
[INFO] [add_two_ints_client]: Result: 25
```

### Example 7: Multi-Node System

This example combines multiple nodes: a sensor publisher, an aggregator, and a display subscriber.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import random


class SensorNode(Node):
    """Publishes raw sensor data."""

    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(Float64, 'raw_data', 10)
        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        msg = Float64()
        msg.data = random.uniform(0.0, 100.0)
        self.publisher_.publish(msg)


class AggregatorNode(Node):
    """Aggregates sensor data and publishes statistics."""

    def __init__(self):
        super().__init__('aggregator_node')
        self.subscription = self.create_subscription(
            Float64, 'raw_data', self.data_callback, 10
        )
        self.publisher_ = self.create_publisher(String, 'statistics', 10)
        self.data_buffer = []
        self.max_buffer_size = 5

    def data_callback(self, msg):
        self.data_buffer.append(msg.data)

        # Keep only last N readings
        if len(self.data_buffer) > self.max_buffer_size:
            self.data_buffer.pop(0)

        # Calculate and publish average
        if len(self.data_buffer) == self.max_buffer_size:
            avg = sum(self.data_buffer) / len(self.data_buffer)
            stats_msg = String()
            stats_msg.data = f'Avg (last {self.max_buffer_size}): {avg:.2f}'
            self.publisher_.publish(stats_msg)


class DisplayNode(Node):
    """Displays processed statistics."""

    def __init__(self):
        super().__init__('display_node')
        self.subscription = self.create_subscription(
            String, 'statistics', self.display_callback, 10
        )

    def display_callback(self, msg):
        self.get_logger().info(f'DISPLAY: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    # Create all nodes
    sensor = SensorNode()
    aggregator = AggregatorNode()
    display = DisplayNode()

    # Create executor to manage all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sensor)
    executor.add_node(aggregator)
    executor.add_node(display)

    try:
        executor.spin()
    finally:
        sensor.destroy_node()
        aggregator.destroy_node()
        display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts:**
- **Multi-node architecture**: Three nodes with distinct responsibilities
- **Data pipeline**: Sensor → Aggregator → Display
- **Line 69**: `MultiThreadedExecutor` runs multiple nodes in one process
- **Real-world pattern**: Mirrors perception → processing → visualization pipelines

**Expected Output:**
```
[INFO] [display_node]: DISPLAY: Avg (last 5): 52.34
[INFO] [display_node]: DISPLAY: Avg (last 5): 48.91
[INFO] [display_node]: DISPLAY: Avg (last 5): 55.67
...
```

## Message Types

ROS 2 uses **strongly-typed messages** for all communication. Every topic and service has a defined message type that specifies the data structure, ensuring type safety across the system.

### Standard Message Types

The `std_msgs` package provides primitive types:
- `std_msgs/String`: Text data
- `std_msgs/Float64`, `Float32`: Floating-point numbers
- `std_msgs/Int32`, `Int64`: Integers
- `std_msgs/Bool`: Boolean values
- `std_msgs/Header`: Timestamp and frame metadata

### Common Interface Types

The `common_interfaces` packages provide domain-specific messages:
- `sensor_msgs`: LaserScan, Image, Imu, NavSatFix
- `geometry_msgs`: Point, Pose, Twist, Transform
- `nav_msgs`: Odometry, Path, OccupancyGrid
- `example_interfaces`: AddTwoInts (service), SetBool (service)

### Custom Message Types

You can define custom messages in your package:

```
# my_package/msg/RobotStatus.msg
string robot_name
float64 battery_voltage
bool is_moving
geometry_msgs/Pose current_pose
```

After building, import with:
```python
from my_package.msg import RobotStatus
```

### Message Inspection

View message structure:
```bash
ros2 interface show std_msgs/msg/String
ros2 interface show sensor_msgs/msg/LaserScan
```

Output:
```
# std_msgs/String
string data

# sensor_msgs/LaserScan
std_msgs/Header header
float32 angle_min
float32 angle_max
float32[] ranges
...
```

### Best Practices

1. **Use standard types when possible**: Promotes interoperability
2. **Namespace custom messages**: Prevents conflicts (`my_robot/Status` not `Status`)
3. **Include timestamps**: Add `std_msgs/Header` for time-sensitive data
4. **Document units**: Comment field units (meters, radians, etc.)
5. **Version carefully**: Changing message definitions breaks compatibility

## Hands-On Exercises

### Exercise 1: Publisher-Subscriber Pair

**Goal**: Create a node that publishes your name every 2 seconds and a subscriber that greets you.

**Steps**:
1. Create a publisher node that sends `std_msgs/String` with your name
2. Set timer period to 2.0 seconds
3. Create a subscriber that prints "Hello, [name]!"
4. Run both nodes and verify the greeting appears

**Success Criteria**: Terminal shows "Hello, [YourName]!" every 2 seconds

### Exercise 2: Sensor Data Processing

**Goal**: Modify the temperature monitor to calculate a running average.

**Steps**:
1. Add a list to store the last 5 temperature readings
2. Calculate the average in the callback
3. Log both the current temperature and the running average
4. Test with the temperature sensor publisher

**Success Criteria**: Output shows both instantaneous and average temperatures

### Exercise 3: Service Configuration

**Goal**: Create a service that configures the temperature monitor's thresholds.

**Steps**:
1. Define a custom service type with `float64 min_temp` and `float64 max_temp` in the request
2. Add a service server to the TemperatureMonitor node
3. Create a client that calls this service
4. Verify the thresholds change dynamically

**Success Criteria**: Alerts trigger at new thresholds after service call

### Exercise 4: Multi-Publisher System

**Goal**: Create a system with 3 sensor publishers and 1 aggregator.

**Steps**:
1. Create 3 temperature sensor nodes publishing to different topics (`temp1`, `temp2`, `temp3`)
2. Create an aggregator that subscribes to all 3 topics
3. Aggregate should publish the maximum temperature among all sensors
4. Use `MultiThreadedExecutor` or run in separate processes

**Success Criteria**: Aggregator correctly identifies and publishes the highest temperature

### Exercise 5: Topic Inspection

**Goal**: Use ROS 2 command-line tools to inspect your running system.

**Steps**:
1. Run the temperature sensor and monitor nodes
2. Use `ros2 node list` to see active nodes
3. Use `ros2 topic list` to see active topics
4. Use `ros2 topic echo /temperature` to view live data
5. Use `ros2 topic hz /temperature` to measure publish rate

**Success Criteria**: You can identify all nodes, topics, and verify the 2 Hz publish rate

## Key Takeaways

- **Nodes** are independent processes that perform specific tasks. Structure your system into logical nodes based on update rates, failure domains, and reusability.

- **Topics** implement asynchronous publish-subscribe for continuous data streams. Use them for sensor data, status updates, and commands where producers don't need acknowledgment.

- **Services** implement synchronous request-response for queries and configuration. Use them when you need an immediate answer or want to trigger a computation.

- **Message types** ensure type safety. Use standard types when possible and create custom messages for domain-specific data.

- The `rclpy.node.Node` class is the foundation of every Python node, providing publishers, subscribers, services, timers, and logging.

- **Decoupling** is key: publishers don't know about subscribers, enabling dynamic system composition and fault isolation.

- **Quality of Service (QoS)** policies let you tune reliability, durability, and history for each topic based on your application's needs.

---

**Previous**: [ROS 2 Fundamentals](./ros2-fundamentals.md)
