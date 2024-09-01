
# Sztuczna Inteligencja w Robotyce - projekt zaliczeniowy

Niniejsze repozytorium przedstawia estymację pozycji robota jednoosiowego z dwoma kołami z pomocą faktografu oraz z wykorzystaniem Nav2 i GTSAM. Zgodnie z wymaganiami w projekcie wykorzystano do estymacji pozycji sensory GPS, IMU oraz Odometrię.

## Uruchomienie

### Zależności:

```
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation
sudo apt-get install ros-humble-rqt-robot-steering
sudo apt install ros-humble-robot-localization
```

### Uruchomienie:

1. Upewnij się, że masz zainstalowane wszystkie niezbędne, podane wyżej zależności:
   ```bash
   sudo apt update
   sudo apt upgrade
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation
   sudo apt-get install ros-humble-rqt-robot-steering
   sudo apt install ros-humble-robot-localization
   ```

2. Sklonuj repozytorium:
   ```bash
   git clone https://github.com/AdamTheStudent/SIwR_robot_sim.git
   cd SIwR_robot_sim
   ```

3. Zbuduj projekt:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Uruchom symulację:
   ```bash
   ros2 launch basic_mobile_robot basic_mobile_bot_v3.launch.py 
   ```
5. Uruchom wizulizację wykresu estymowanego położenia
   ```bash
   ros2 launch basic_mobile_robot pos_estimation.launch.py
   ```
6. Uruchom narzędzie do sterowania symulowanym robotem:
   ```bash
   ros2 run rqt_robot_steering rqt_robot_steering --force-discover
   ```
7. Efekt uruchomienia powinien wyglądać następująco:

![Demo symulacji](https://github.com/AdamTheStudent/SIwR_robot_sim/blob/main/img/gif.gif)

## Wykresy przedstawiające dane z sensorów i estymowaną pozycję robota

![Przyklad 1](https://github.com/AdamTheStudent/SIwR_robot_sim/blob/main/img/Plot.png)
![Przyklad 2](https://github.com/AdamTheStudent/SIwR_robot_sim/blob/main/img/Plot2.png)
![Przyklad 3](https://github.com/AdamTheStudent/SIwR_robot_sim/blob/main/img/Plot3.png)

## Opis szczegółowy projektu i podsumowanie

Pierwszą częścią projektu było przygotowanie symulacji robota o dwóch kołach z użyciem sensorów oraz środowiska zasymulowanego w Gazebo. 
Do przygotowania symulacji zachowania sensorów i rozszerzonego filtu Kalmana (EKF) wykorzystano framework Nav2.

Następnie przygotowana została estymacja pozycji robota w otaczającym go środowisku z wykorzystaniem GTSAM.
Do estymacji pozycji wykorzystano dodatkowo sensory GPS oraz Odometrię zwróconą z EKF, który wykorzystuje połączenie
sensorów IMU i Odometrii z enkoderów.

### Wyjaśnienie wybranych fragmentów projektu

#### Odpowiedzialne za estymację pozycji robota są funkcje zaimplementowane w `pos_estimation.py`:

Inicjalizacja zmiennych globalnych, używanych do przechowywania danych GPS i odometrii, jak również do zapisywania współrzędnych w formacie UTM. graph to nieliniowy graf, w którym gromadzimy czynniki pozycji (z GPS i odometrii), a initial_estimate przechowuje wstępne estymacje pozycji robota:

```python
gps_data = None
odom_data = None

gps_x = []
gps_y = []
odom_x = []
odom_y = []

initial_utm_x = None
initial_utm_y = None
pose_index = 0
graph = gtsam.NonlinearFactorGraph()
initial_estimate = gtsam.Values()
estimate_x = []
estimate_y = []
```

Klasa SensorListener tworzy węzeł ROS, który subskrybuje dwa tematy: /gps/fix (dla danych GPS) i /odometry/filtered (dla danych odometrii). Funkcje gps_callback oraz odom_callback zapisują odebrane wiadomości w globalnych zmiennych gps_data i odom_data:

```python
class SensorListener(Node):

    def __init__(self):
        super().__init__('sensor_listener')
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)

    def gps_callback(self, msg):
        global gps_data
        gps_data = msg

    def odom_callback(self, msg):
        global odom_data
        odom_data = msg
```

Funkcja update_data() zbiera dane z GPS i odometrii, przekształca współrzędne GPS na układ UTM, a następnie zapisuje współrzędne lokalne (po przekształceniu do układu odniesienia UTM). Dodaje nowe informacje do grafu optymalizacyjnego GTSAM i aktualizuje wstępne estymacje pozycji robota. Co 0.1 sekundy następuje zebranie nowych danych:
```python
def update_data():
    global gps_data, odom_data, initial_utm_x, initial_utm_y, pose_index

    while rclpy.ok():
        if gps_data:
            utm_coords = utm.from_latlon(gps_data.latitude, gps_data.longitude)
            
            if initial_utm_x is None and initial_utm_y is None:
                initial_utm_x = utm_coords[0]
                initial_utm_y = utm_coords[1]
            local_utm_x = utm_coords[0] - initial_utm_x
            local_utm_y = utm_coords[1] - initial_utm_y

            gps_x.append(local_utm_x)
            gps_y.append(local_utm_y)
            gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([.8, .8, 0.1]))
            graph.add(gtsam.PriorFactorPose2(sym.X(pose_index), gtsam.Pose2(local_utm_x, local_utm_y, 0), gps_noise))

        if odom_data:
            local_odom_x = odom_data.pose.pose.position.x
            local_odom_y = odom_data.pose.pose.position.y
            odom_x.append(local_odom_x)
            odom_y.append(local_odom_y)

            if pose_index > 0:
                odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([.2, .2, 0.1]))
                odometry = gtsam.Pose2(local_odom_x - odom_x[-2], local_odom_y - odom_y[-2], 0)
                graph.add(gtsam.BetweenFactorPose2(sym.X(pose_index - 1), sym.X(pose_index), odometry, odom_noise))
            initial_estimate.insert(sym.X(pose_index), gtsam.Pose2(local_odom_x, local_odom_y, 0))
            pose_index += 1

        time.sleep(0.1)
```

Funkcja optimize_and_plot_result() przeprowadza optymalizację pozycji robota przy użyciu algorytmu Gaussa-Newtona z GTSAM, a następnie aktualizuje listy estymowanych współrzędnych (estimate_x i estimate_y). Wyniki optymalizacji są iteracyjnie wyświetlane co 1 sekundę:
```python
def optimize_and_plot_result():
    global pose_index

    while rclpy.ok():  
        if pose_index > 0:
            optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate)
            result = optimizer.optimize()

            estimate_x.clear()
            estimate_y.clear()

            for i in range(pose_index):
                key = sym.X(i)
                if result.exists(key): 
                    pose = result.atPose2(key)
                    estimate_x.append(pose.x())
                    estimate_y.append(pose.y())

        time.sleep(1)
```

Finalnie program inicjalizuje ROS2, tworzy węzeł subskrybujący dane GPS i odometrii, a następnie uruchamia wątki do zbierania danych, optymalizacji i wizualizacji wyników. Program kończy się zamknięciem ROS2 i zniszczeniem węzła.

## Struktura Factor Graph

Przyjęta struktura Factor Graph'ów do estymowania kolejnych pozycji jest następująca:
```
(X0) -- [EKF] -- (X1) -- [EKF] -- (X2) -- [EKF] -- (X3)
 |                |                |
[GPS]           [GPS]            [GPS]
```
Sygnały z sensorów i estymowana pozycja są plotowane w czasie rzeczywistym na wspólnym wykresie.

## Autorzy

- [@AdamTheStudent](https://www.github.com/AdamTheStudent)
- [@danak212](https://github.com/danak212)

## Literatura

[GTSAM](https://gtsam.org/) 

[Nav2](https://docs.nav2.org/setup_guides/odom/setup_odom.html)

[Symulacja robota](https://automaticaddison.com/)
