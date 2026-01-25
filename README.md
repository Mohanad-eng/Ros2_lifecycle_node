# ROS 2 Lifecycle Node – States Overview

This repository explains the **ROS 2 Lifecycle Node** concept, focusing on its **main states**, transitions, and behavior.  
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
terminal

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

