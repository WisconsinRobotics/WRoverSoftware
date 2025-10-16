nmcli radio wifi on
nmcli dev wifi connect WRoverBasestation_5G password "i#3Er0b0"

until sshpass -p i#3Er0b0 ssh wiscrobo@192.168.1.134; do
    sleep 3
done

cd /home/wiscrobo/workspace/WRoverSoftware
git checkout dev/roverStation

{
    colcon build &&
} || {
    rm -r build install log && colcon build
}

./roverStart.sh