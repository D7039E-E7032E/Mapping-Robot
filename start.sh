while :
do
rosrun map_server map_saver -f home/vision/Documents/Maps #Saves map to a certain dir
scp map.pgm map.yaml coarl@192.168.1.97:~/Documents/Maps #Transfers maps to central computer
sudo rm ~/Documents/Maps/map.pgm #Removes transfered files
sudo rm ~/Documents/Maps/map.yaml
sleep 10 #Waits
done
