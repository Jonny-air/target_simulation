#!/bin/bash
cat << "EOF"
Installing Node Package Manager, Node.js and GUI dependencies...
EOF

sudo apt install npm
sudo npm install -g npm@latest
npm install -g n
sudo n stable
sudo npm install -g expo-cli
npm install react-ros
sudo apt-get install ros-melodic-rosbridge-suite
npm install roslib
cd ~/catkin_ws/src/target_simulation/target_control
npm install --force
npm run build


cat << "EOF"
Finished. Don't forget to source!
You can now start the gui with:

serve -s build
or
npm start

EOF
