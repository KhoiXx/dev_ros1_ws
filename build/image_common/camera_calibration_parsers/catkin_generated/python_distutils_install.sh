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

echo_and_run cd "/home/khoixx/dev_ros1_ws/src/image_common/camera_calibration_parsers"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/khoixx/dev_ros1_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/khoixx/dev_ros1_ws/install/lib/python2.7/dist-packages:/home/khoixx/dev_ros1_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/khoixx/dev_ros1_ws/build" \
    "/usr/bin/python2" \
    "/home/khoixx/dev_ros1_ws/src/image_common/camera_calibration_parsers/setup.py" \
     \
    build --build-base "/home/khoixx/dev_ros1_ws/build/image_common/camera_calibration_parsers" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/khoixx/dev_ros1_ws/install" --install-scripts="/home/khoixx/dev_ros1_ws/install/bin"
