cd /home/wiscrobo/workspace/WRoverSoftware

if [ $1 == 'p' ]; then
  git checkout dev/basestation
fi

colcon build
./basestation.sh