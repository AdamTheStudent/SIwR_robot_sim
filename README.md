Uruchomienie:
git clone https://github.com/AdamTheStudent/SIwR_robot_sim.git
ros2 launch basic_mobile_robot basic_mobile_bot_v3.launch.py <- uruchomienie symulacji robota
ros2 run rqt_robot_steering rqt_robot_steering --force-discover <- panel sterowania robotem
ros2 launch basic_mobile_robot pos_estimation.launch.py <- plotowanie surwoych danych z sensorow #TODO estymowanie za pomoca factor graphs

Source:
https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/

Potrzebne do uruchomienia:
sudo apt-get install ros-humble-rqt-robot-steering
sudo apt install ros-humble-robot-localization < nie jestem pewien >
jezeli cos jeszcze jest potrzebne do uruchomienia sprawdzic na stronie na ktorej sie wzorowalem.

