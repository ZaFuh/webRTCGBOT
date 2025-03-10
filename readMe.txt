How to run this:

1. Set up the signalling server on a new terminal:

cd /home/hady/teleop/webrtc-signal-server
node index.js 

If encoutering errors, run this:
npm install ws
nvm install --lts
nvm use --lts

2. On another terminal, run the ros thing:
source /home/hady/teleop/gbot_ws/install/setup.bash
ros2 run teleop image_subscriber

build the package if needed:
cd /home/hady/teleop/gbot_ws
rm -rf build/teleop
colcon build --packages-select teleop

3. Run the reciver script (gbot_teleop directory):
npm i
npm run dev

Still hasnt stream anything from the webcam yet, but this is progress :D