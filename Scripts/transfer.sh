cd ~/catkin_ws/
source devel/setup.bash
rosrun map_server map_saver -f ~/Documents/Maps/map #Saves map to a certain dir
scp ~/Documents/Maps/map.pgm ~/Documents/Maps/map.yaml coarl@192.168.1.97:~/Documents/Maps/transferedMaps #Transfers maps to central computer
#sudo rm ~/Documents/Maps/map.pgm #Removes transfered files
#sudo rm ~/Documents/Maps/map.yaml

