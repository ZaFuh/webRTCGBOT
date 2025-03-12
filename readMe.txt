How to run this:


STEP1: set web signaling server 
cd /home/hady/teleop/webrtc-signal-server 
node index.js 

If encoutering errors, run this:
npm install ws
nvm install --lts
nvm use --lts

STEP 2:  Set receiver script (gbot-teleop) and open the local host url
cd /home/hady/teleop/gbot-teleop
npm i
npm run dev

STEP 3:  For USB webcam(make sure its connected) run camera topic on a new terminal
cd /home/hady/teleop/gbot_ws
ros2 run usb_cam usb_cam_node_exe

check camera topics if needed:
ros2 topic list

STEP 4: RUn the image subscriber with show camerra flag on another terminal
cd /home/hady/teleop/gbot_ws
ros2 run teleop image_subscriber --ws-url ws://localhost:8080 --show-camera

build the package if needed:
cd /home/hady/teleop/gbot_ws
rm -rf build/teleop
colcon build --packages-select teleop

Change camera topic on Line 71 IF Needed, then build and source again.

yOU'RE WELCOME
MADE BY ZAHRA

