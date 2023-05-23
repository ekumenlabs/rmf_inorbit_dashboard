# Docker setup

The adapter can be run in a docker container based on `osrf/ros:humble-desktop` that simplifies the process of installing dependencies not available from rosdep, and also includes useful utilities.

## Build the docker image

Run the following script to build the docker image

```
./docker/build.sh
```

## Run the container

Run the following script to run the container and mount the workspace on it

```
./docker/run.sh
```

The default entrypoint is a bash shell with the ROS workspace (`~/ws`) as the default location. The user will have the same name as the user that executed the run script, with passwordless sudo.

## Commit changes to the image

You can commit changes to the image when you exit the container. There is an exit trap implemented. For example, assuming you are running a terminal in the container:

```
$ exit
exit
access control enabled, only authorized clients can connect
Do you want to overwrite the image called 'inorbit_rmf' with the current changes? [y/n]: y
Overwriting docker image...
sha256:d7bc07a09aadb9b1935b5c0a87e64ad19c5245c9145fe5b39ef6b198be6cdcc1
```

In case you applied changes you don't want to preserve, just type `n`. Otherwise, `y`.

## Using different image or container names

In case you want to change the image or container name, make sure to run the scripts with `-h` or `--help` to see all the possible options.

```
$ ./docker/build.sh --help
Building the docker image for ros2 humble rmf development.

Usage:   build.sh [OPTIONS]

  Options:

        -i --image_name          Name of the image to be built (default inorbit_rmf).

  Example:

        build.sh --image_name custom_image_name

$ ./docker/run.sh --help
Running the container...

Usage:   run.sh [OPTIONS]

  Options:

        -i --image_name          Name of the image to be run (default inorbit_rmf).

        -c --container_name      Name of the container(default inorbit_rmf).

        --use_nvidia             Use nvidia runtime.

  Examples:

        run.sh

        run.sh --image_name custom_image_name --container_name custom_container_name
```
