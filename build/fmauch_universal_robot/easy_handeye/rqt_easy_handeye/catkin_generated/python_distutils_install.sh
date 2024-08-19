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

echo_and_run cd "/home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/easy_handeye/rqt_easy_handeye"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/cn/NeuralSymbol_AI_for_task2/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cn/NeuralSymbol_AI_for_task2/install/lib/python3/dist-packages:/home/cn/NeuralSymbol_AI_for_task2/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cn/NeuralSymbol_AI_for_task2/build" \
    "/home/cn/anaconda3/envs/NSAI/bin/python3" \
    "/home/cn/NeuralSymbol_AI_for_task2/src/fmauch_universal_robot/easy_handeye/rqt_easy_handeye/setup.py" \
    egg_info --egg-base /home/cn/NeuralSymbol_AI_for_task2/build/fmauch_universal_robot/easy_handeye/rqt_easy_handeye \
    build --build-base "/home/cn/NeuralSymbol_AI_for_task2/build/fmauch_universal_robot/easy_handeye/rqt_easy_handeye" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/cn/NeuralSymbol_AI_for_task2/install" --install-scripts="/home/cn/NeuralSymbol_AI_for_task2/install/bin"
