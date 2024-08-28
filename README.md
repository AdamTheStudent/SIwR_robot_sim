
# Projekt Sztuczna Inteligencja w Robotyce

Projekt estymujacy pozycje robota dwukolowego za pomoca Factor Graphs  z wykorzystaniem Nav2 i GTSAM. Zasymulowane i wykorzystane do estymacji zostaly sensory GPS, IMU i Odometria z enkoderow. 


## Usage

Zaleznosci:

```
sudo apt-get install ros-humble-rqt-robot-steering
sudo apt install ros-humble-robot-localization
```
Uruchomienie:
```
git clone https://github.com/AdamTheStudent/SIwR_robot_sim.git

ros2 launch basic_mobile_robot basic_mobile_bot_v3.launch.py 
# Symulacja robota i sensotow
ros2 launch basic_mobile_robot pos_estimation.launch.py 
# Plot z estymowanym polozeniem
ros2 run rqt_robot_steering rqt_robot_steering --force-discover
# Sterowanie robotem
```

## Screenshots

![Przyklad 1](https://github.com/AdamTheStudent/SIwR_robot_sim/blob/main/img/Plot.png)
![Przyklad 2](https://github.com/AdamTheStudent/SIwR_robot_sim/blob/main/img/Plot2.png)
![Przyklad 3](https://github.com/AdamTheStudent/SIwR_robot_sim/blob/main/img/Plot3.png)

## Related

Przydatne strony:

[GTSAM](https://gtsam.org/) 

[Nav2](https://docs.nav2.org/setup_guides/odom/setup_odom.html)

[Symulacja robota](https://automaticaddison.com/)

## Opis projektu i podsumowanie

Pierwsza czescia projektu bylo przygotowanie symulacji robota dwukolowego sensorow oraz swiata w symulatorze Gazebo.
Do przygotowania symulacji zachowania sensorow i rozszerzonego filtu Kalmana (EKF) wykorzystano framework NAV2.

Nastepnie przygotowalismy projekt estymujacy pozycje robota w swiecie przy uzyciu GTSAM.
Do estymacji pozycji wykorzystano sensory GPS oraz Odometrie zwrocona z EKF (extended kalman filter), ktory wykorzystuje senory IMU oraz Odometrie z enkoderow.

Przyjeta struktura Factor Graphow do estymowania kolejnych pozycji ma strukture:
```
(X0) -- [EKF] -- (X1) -- [EKF] -- (X2) -- [EKF] -- (X3)
 |                |                |
[GPS]           [GPS]            [GPS]
```
Sygnaly z sensorow i estymowana pozycja jest plotowana w czasie rzeczywistym na wspolnym wykresie.

## Authors

- [@AdamTheStudent](https://www.github.com/AdamTheStudent)
- [@danak212](https://github.com/danak212)

## Demo

Linki do YouTube / .gif #TODO

