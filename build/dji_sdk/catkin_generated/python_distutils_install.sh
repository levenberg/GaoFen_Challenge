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

echo_and_run cd "/root/Documents/roswork/GaoFen_Challenge/src/dji_sdk"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/root/Documents/roswork/GaoFen_Challenge/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/root/Documents/roswork/GaoFen_Challenge/install/lib/python2.7/dist-packages:/root/Documents/roswork/GaoFen_Challenge/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/root/Documents/roswork/GaoFen_Challenge/build" \
    "/usr/bin/python" \
    "/root/Documents/roswork/GaoFen_Challenge/src/dji_sdk/setup.py" \
    build --build-base "/root/Documents/roswork/GaoFen_Challenge/build/dji_sdk" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/root/Documents/roswork/GaoFen_Challenge/install" --install-scripts="/root/Documents/roswork/GaoFen_Challenge/install/bin"
