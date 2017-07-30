# nao_webots
An interface between ROS and Webots for the NAO robot

## Requirements
* Webots
* [`webots_run`](http://github.com/mmattamala/webots_run.git) package

## Preliminaries
We must set some environment variables before:
* Set `WEBOTS_HOME` to the folder where you put Webots. Example: `WEBOTS_HOME=/home/matias/webots`
* Add `WEBOTS_HOME` to the `PATH`. Example: `PATH=${PATH}:${WEBOTS_HOME}`

## Compilation and Installation
Since everything is _catkinized_, `catkin_make` is enough to build everything. However, you **must copy** the executable generated by catkin (i.e. `nao_webots_node`) into the Webots controller folder. Particularly, you must copy the executable to `$WEBOTS_HOME/projects/robots/aldebaran/controllers/nao_webots_node` in order to let Webots to find the controller.

## Running the nodes
It is **highly** recommended to use a launch file, you can use `run.launch` as an example. The `webots_run` package will allow you to execute Webots _within_ ROS, and automatically will use the controller set in the `world` file passed as argument to Webots(`nao_demo.wbt` by default).
