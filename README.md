# Project-Algernon
CSCI 3302 Final Project

## What is Project Algernon?

Project Algernon is a three-step robot that maps, path plans, and traverses a pre-made map in WeBots.

### Built With

* Python
* WeBots Application Environment
* Thiago Robot - [Within WeBots Environment]
* Lidar Camera - [Within WeBots Environment]

## Flow of Project

### Step 1: Mapping

Initially, the user of the project controls and guides the robot around the maze so that the robot can utilize the Lidar camera and map the walls and paths.

#### Inital Map Made
![Screen Shot 2021-04-28 at 9 20 39 PM](https://user-images.githubusercontent.com/44630596/134400862-66e0dabd-26f6-4627-9ae6-3fc9877090e7.png)

#### Normalized Map
![Screen Shot 2021-04-28 at 9 21 54 PM](https://user-images.githubusercontent.com/44630596/134400903-e48d38e5-850a-4f64-91f8-435af10ffeaf.png)

### Step 2: Path Planning

Then, utilizing the A* search algorithm, the robot searches and find the optimal path to the goal.

#### Mapped Path
![Screen Shot 2021-04-28 at 9 42 11 PM](https://user-images.githubusercontent.com/44630596/134400934-7c85f468-23d0-48dd-8ba6-7daeacd6aaf1.png)


### Step 3: Traverse

Finally, utilizing inverse kinematics the robot traverses the maze while simuletransely adjusting its movements to avoid the walls and take the proper turns.


## Author

Josef May
