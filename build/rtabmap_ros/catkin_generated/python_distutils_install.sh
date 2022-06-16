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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/sahil/DorleCo/src/slam/rtabmap_ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sahil/DorleCo/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sahil/DorleCo/install/lib/python3/dist-packages:/home/sahil/DorleCo/build/rtabmap_ros/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sahil/DorleCo/build/rtabmap_ros" \
    "/usr/bin/python3" \
    "/home/sahil/DorleCo/src/slam/rtabmap_ros/setup.py" \
    egg_info --egg-base /home/sahil/DorleCo/build/rtabmap_ros \
    build --build-base "/home/sahil/DorleCo/build/rtabmap_ros" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sahil/DorleCo/install" --install-scripts="/home/sahil/DorleCo/install/bin"
