#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/robot/Mapping-Robot/catkin_ws/src/turtlebot3/turtlebot3_example"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/robot/Mapping-Robot/catkin_ws/install_isolated/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/robot/Mapping-Robot/catkin_ws/install_isolated/lib/python2.7/dist-packages:/home/robot/Mapping-Robot/catkin_ws/build_isolated/turtlebot3_example/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/robot/Mapping-Robot/catkin_ws/build_isolated/turtlebot3_example" \
    "/usr/bin/python2" \
    "/home/robot/Mapping-Robot/catkin_ws/src/turtlebot3/turtlebot3_example/setup.py" \
    build --build-base "/home/robot/Mapping-Robot/catkin_ws/build_isolated/turtlebot3_example" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/robot/Mapping-Robot/catkin_ws/install_isolated" --install-scripts="/home/robot/Mapping-Robot/catkin_ws/install_isolated/bin"
