# About

ROS node for ROS-REST interface<br>
As an example, the node registers and publishes to several ROS topics while exposing a REST API to interact with these topics

# How to use

1. install ros and catkin workspace:
    
    if not already installed, install ros from [here](http://wiki.ros.org/noetic/Installation/Ubuntu#Installation)


2. Clone this repo.

   ```terminal
   git clone https://github.com/neriyashul/Bluewhite_Assignment.git
   ```


3. Install the package

  - Option 1: Enter to the directory and run (you might run to enter you password to install the   requirements):

    ```terminal
    source install.sh <path_of_catkin_workspace>
    ```
  - Option 2: 
    * a. copy 'ros_rest_interface' directory to <path_of_catkin_workspace>/src.
    * b. install requirement: 
      ```terminal
      sudo apt-get install libcpprest-dev
      ```
    * c. source setup:
        ```terminal
        cd <path_of_catkin_workspace>
        source devel/setup.bash
        ```
    * d. catkin_make
 
4. Run the node using:

  ```terminal
    roslaunch ros_rest_interface ros_rest_interface.launch
  ```


# Design & Architecture

In this section, I will explain the design and architecture of the project. The classes have been designed with the principles of clean code, SOLID, and KISS in mind. The goal was to keep the classes focused on a single responsibility while keeping the design simple and avoiding unnecessary complexity.

## Main Class: `ros_rest_relay_node`

The `ros_rest_relay_node` class serves as the entry point for the functionality and is responsible for initializing the other classes.

- `web_server`: This class is responsible for managing everything related to the network. It has a field of type `controller_interface`, which it utilizes to handle the requests. 

- The `PointController` class is an implementation of the `controller_interface` specifically tailored to manage requests pertaining to points. 
The `PointController` does not directly send or retrieve data to or from topics. Instead, it interacts with a `RosMessageGateway` to handle this process. The `RosMessageGateway` acts as an intermediary for data transmission and retrieval in the `PointController` system. 


- By separating the server from the controller and data transmission, the code gains advantages in terms of scalability, flexibility, modularity, and testability. This design choice allows for easy adaptation of the server to different projects since ROS and the server are independent of each other. It also facilitates seamless modification of message types by offering alternative implementations of the `controller_interface`.


## Challenges and Solutions

During the development process, certain challenges were encountered, and corresponding solutions were implemented. Here are a couple of examples:

- **Challenge**: Handling the concurrent nature of the server using Casablanca (cpprestsdk) posed a challenge in keeping track of incoming publishers and receivers while ensuring thread-safety in the data structure used. To address this, a `concurrent_unordered_map` was implemented, which combines an unordered_map with mutexes to restrict access and ensure thread-safety. Additionally, the shared_mutex feature of C++17 was utilized to allow multiple readers without restricting access.

- **Challenge**: Reducing the complexity of thread synchronization was a priority, and it was decided to use a single main thread for both ROS and the server. However, a potential problem arose when it was discovered that ROS's thread would sleep for 100ms when there was no job to do. To mitigate this, a thorough investigation was conducted by examining the source code, revealing that both frameworks utilize boost's threads. Testing confirmed that the threads did not interfere with each other adversely.

## Assumptions & Decisions

During the development process, certain assumptions were made, and decisions were taken to ensure the effectiveness of the implementation. These include:

- The URLs and names of the default topics to subscribe to are kept in a YAML configuration file. This allows for easy modification without the need to change the code directly. The configuration file provides a centralized location for finding and modifying these values.

- If a GET request is made for data from a topic that has not received any data yet, the server will return a 'not exists' message. This message contains points with values set to minus infinity, indicating that the requested data does not exist.

- Before publishing a message, the publisher checks if there are any subscribers. If there are no subscribers, the message is not sent. This approach follows a recommendation from ROS.

- In the case of an error, the server sends an appropriate status code and corresponding message to indicate the error to the client.
