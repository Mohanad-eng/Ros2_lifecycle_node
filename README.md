# ROS 2 Lifecycle Node – States Overview

This repository explains the **ROS 2 Lifecycle Node** concept, focusing on its **main states**, transitions, and behavior, containing Full project links to make on lifecycle nodes.
  
Lifecycle nodes are useful when you need **controlled startup, shutdown, and resource management**.

---

## 📌 What is a Lifecycle Node?

A **Lifecycle Node** in ROS 2 is a managed node that follows a predefined set of states.  
Unlike normal nodes, it allows you to explicitly control **when resources are created, activated, deactivated, or destroyed**.

---
## Standard vs. lifecycle nodes

**Standard node** go from unactive to active.

**Lifecycle node** has four states.

---

## 🔄 Lifecycle States

A ROS 2 lifecycle node has **four main states**:

Each state represents a **different level of readiness**.

---

## 1️⃣ Unconfigured State

### Description
- The node **has been created**
- No configuration has been done yet
- This is the **initial state**

### What happens here?
- `on_configure()` has **NOT** been called
- No publishers, subscribers, services, timers, or hardware interfaces exist

### Example Use
- Node is waiting to be configured
- Parameters may be declared but not used

📎 *Think of this as: the node exists, but nothing is prepared.*

---

## 2️⃣ Inactive State

### Description
- The node has been **configured successfully**
- It is ready to run, but **not doing anything yet**

### What happens here?
- `on_configure()` **HAS** been called
- Publishers and subscribers are created
- Timers exist but are **not running**
- No messages are being published or processed

### Why this state is important?
- Allows checking configuration before running
- Safe place to pause a node

📎 *Think of this as: everything is ready, but the node is paused.*

---

## 3️⃣ Active State

### Description
- The node is **fully running**
- This is the main working state

### What happens here?
- `on_activate()` **HAS** been called
- Publishers and subscribers are active
- Timers start running
- Messages are published and received

### Example Use
- Robot control
- Sensor data publishing
- Navigation and planning

📎 *Think of this as: the node is doing its real job.*

---

## 4️⃣ Finalized State

### Description
- The node is **shut down permanently**
- This is the final state

### What happens here?
- `on_shutdown()` **HAS** been called
- Resources are released
- Publishers and subscribers are destroyed
- The node **cannot be restarted**

📎 *Think of this as: the node is dead.*

---

## 🔁 Lifecycle State Transitions

Lifecycle nodes move between states using **transitions**.

### Typical Flow

```text
Unconfigured
     ↓ configure
Inactive
     ↓ activate
Active
     ↓ deactivate
Inactive
     ↓ shutdown
Finalized
```
## Let's Make an example to understand the lifecycle nodes 
* First make a Workspace called lifecycle_ex1
```

mkdir lifecycle_ex1
```
* second make a directory src and add package called lifecycle_pkg
```
cd lifecycle_ex1

mkdir src

cd src

ros2 create pkg --build-type ament_python lifecycle_pkg
```
* third make a lifecycle node called node_lifecycle
```
cd lifecycle_pkg

cd lifecycle_pkg

touch node_lifecycle 

chmod +x node_lifecycle
```
* open in Vs code or your prefebale IDE and paste this code 
```
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


class SimpleLifecycleNode(LifecycleNode):

    def __init__(self):
        super().__init__('simple_lifecycle_node')
        self.get_logger().info('Node created: Unconfigured state')

    # Called when going from Unconfigured → Inactive
    def on_configure(self, state: State):
        self.get_logger().info('Configuring node...')
        self.get_logger().info('Node configured: Inactive state')
        return TransitionCallbackReturn.SUCCESS

    # Called when going from Inactive → Active
    def on_activate(self, state: State):
        self.get_logger().info('Activating node...')
        self.get_logger().info('Node activated: Active state')
        return TransitionCallbackReturn.SUCCESS

    # Called when going from Active → Inactive
    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating node...')
        self.get_logger().info('Node deactivated: Inactive state')
        return TransitionCallbackReturn.SUCCESS

    # Called when going from Inactive → Unconfigured
    def on_cleanup(self, state: State):
        self.get_logger().info('Cleaning up node...')
        self.get_logger().info('Node cleaned: Unconfigured state')
        return TransitionCallbackReturn.SUCCESS

    # Called when shutting down (Any state → Finalized)
    def on_shutdown(self, state: State):
        self.get_logger().info('Shutting down node...')
        self.get_logger().info('Node finalized')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)

    node = SimpleLifecycleNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
## 🛠 Step 5: Register the Node (VERY IMPORTANT)
* Edit setup.py and add the entry point:
```
bash

entry_points={
    'console_scripts': [
        'node_lifecycle = lifecycle_pkg.node_lifecycle:main',
    ],
},
```
⚠️ Without this step, the node will not run.

## ▶️ Step 6: Build and Run
```
cd ~/lifecycle_ex1
colcon build
source install/setup.bash
ros2 run lifecycle_pkg node_lifecycle
```
## 🎮 Step 7: Control the Lifecycle

Using Ros2 CLI (Command line interface):
```
ros2 lifecycle set <lifecycle node name> command
```
```
ros2 lifecycle set /simple_lifecycle_node configure
```
> the node configured
```
ros2 lifecycle set /simple_lifecycle_node activate
```
> the node activated and publishers and services are running
```
ros2 lifecycle set /simple_lifecycle_node deactivate 
```
> the node deactivated and publishers and services arenot running
```
ros2 lifecycle set /simple_lifecycle_node shutdown
```
> the node shutdown

## Making A publisher and subscriber using Lifecycle nodes



📌 Lifecycle Communication Rules :

1. Publishers and subscribers are created in the **on_configure()** function

2. The node **is not allowed** to communicate before configuration.

3. Publishers must be created using **create_lifecycle_publisher()**

4. Normal publishers (create_publisher()) do not respect lifecycle states.

5. The publisher starts sending data only after **on_activate()** is called

6. Before activation, messages are not published.

## Project 1:

### making a publisher that publish a number 

**Step 1**: Create a workspace and make a src directory
```
mkdir lifecycle_ws

cd lifecycle_ws

mkdir src 
```
---
**Step 2**: Create your package
```
cd src 

ros2 pkg create  --build-type ament_python lifecycle_pkg
```
>This will create a folder lifecycle_pkg with Python structure:
```
lifecycle_pkg/

├── package.xml

├── setup.py

├── resource/

├── lifecycle_pkg/

│   └── __init__.py

└── test/
```
---
**Step 3**: Add your lifecycle node code

> Create a file called lifecyclenode.py inside the package:
```
cd lifecycle_pkg/lifecycle_pkg

touch lifecyclenode.py

chmod +x lifecyclenode.py
```
**Go** inside lifecyclenode.py by using :
```
cd src 

code .
```
>and this will open the whole src in Vscode and you can paste the code
>or you can open the file in the terminal from nano 
```
nano lifecyclenode.py
```
**Then** paste the code and make **ctrl+s** to save and **ctrl+x** to Exit 

Code :
```
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import Int16

class NumberPublisher(LifecycleNode):
    def __init__(self):
        super().__init__('number_publisher_lifecycle')
        self.get_logger().info('Lifecycle node created')
        self.publisher = None
        self.timer = None

    def on_configure(self, state):
        self.get_logger().info('Configuring...')
        self.publisher = self.create_publisher(Int16, 'number_topic', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating...')
        self.timer = self.create_timer(1.0, self.publish_number)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        if self.timer:
            self.timer.cancel()
            self.timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up...')
        self.publisher = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def publish_number(self):
        msg = Int16()
        msg.data = 4
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---
**Step 4**: Edit setup.py to make the node executable

>Open setup.py and make sure you have this entry point:
```
entry_points={
    'console_scripts': [
        'lifecyclenode = lifecycle_pkg.lifecyclenode:main',
    ],
},
```
---
**Step 5**: Build the workspace
```
cd ~/lifecycle_ws

colcon build
```
**Then** source the workspace:
```
source install/setup.bash
```
---
**Step 6**: Run the lifecycle node
```
ros2 run lifecycle_pkg lifecyclenode
```
---
**in another terminal** source the workspace :
```
source ~/lifecycle_ws/install/setup.bash
```
Then **configure** and **activate**:
```
ros2 lifecycle set /number_publisher_lifecycle configure
```
you see in the terminal :
```
Transitioning successful
```
Then 
```
ros2 lifecycle set /number_publisher_lifecycle activate
```
you see in the terminal  :
```
Transitioning successful
```
**Step 7**: Cleanup / deactivate:
```
ros2 lifecycle set /number_publisher_lifecycle deactivate
```
```
ros2 lifecycle set /number_publisher_lifecycle cleanup
```
```
ros2 lifecycle set /number_publisher_lifecycle shutdown
```
