# Krok 0 - Wstępna konfiguracja linuxa
- Pobierz i zainstaluj swoje ulubione narzędzia, np:
```bash
sudo apt update && sudo apt install -y \
	curl \
	fzf \
	git \
	gpg \
	htop \
	mc \
	neovim \
	ranger \
	tmux \
	wget
```
- Visual Studio Code [docs](https://code.visualstudio.com/docs/setup/linux)
```bash
sudo apt install -y wget gpg apt-transport-https
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg
sudo apt update
sudo apt install code
```

- Zainstaluj ROS 2 Humble [docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

    1. Upewnij się, że lokalizacja systemu jest prawidłowo ustawiona:
    ```bash
	locale  # check for UTF-8

	sudo apt update && sudo apt install locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

	locale  # verify settings
	``` 
	2. Dodaj repozytorium PPA do systemu:
    ```bash
	sudo apt install -y software-properties-common curl
	sudo add-apt-repository universe
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	```
	3. Zainstaluj paczki ROS 2:
    ```bash
	sudo apt update
	sudo apt install -y ros-humble-desktop
	```

	4. Pamiętaj, aby za każdym razem w terminalu użyć komendy `source` z odpowiednim workspace'em, np. `source /opt/ros/humble/setup.bash` 


## Instalacja narzędzi do budowy projektów

Proces instalacji narzędzi należy zacząc od `rosdep` - służy do automatycznego instalowania brakujących zalezności sklonowanych repozytoriów (paczek ROS)
```bash
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade -y
```

Następnie należy zainstalować narzędzie budowy paczek `colcon` z dodatkiem do `mixin` (który służy do rozwikłania dziedziczenia zależności paczek):
```bash
sudo apt install -y python3-colcon-common-extensions python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

Kolejnym narzędziem jest `vcstool`, służącym do automatycznego klonowania repozytoriów przy pomocy systemu kontroli wersji (np. git).
```bash
sudo apt install -y python3-vcstool
```

## Instalacja MoveIt
Zważająć na fakt, że ROS 2 wciąż jest prężnie rozwijany, nie wszystkie paczki i moduły mogą działać od razu po ich instalacji z oficjalnego repozytorium PPA. Z tego powodu, na potrzeby ćwiczenia labolatoryjnego, nalezy pobrać i samodzielnie zbudować paczki odpowiedzialne za moduł MoveIt.

1. Procedurę zaczynamy od pobrania paczki `moveit2_tutorials`:
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros-planning/moveit2_tutorials --branch humble --depth 1
```
2. Nastepnie kolonujemy brakujące paczki przy pomocy narzędzia `vcstool`:
```bash
vcs import < moveit2_tutorials/moveit2_tutorials.repos
```
3. Zainstalować brakuje zależności (wszystkich paczek) przy pomocy `rosdep`:
```bash
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```
4. Kolejnym krokiem jest budowa paczek *MoveIt* i wczytania ich do aktualnego obszaru roboczego:
```bash
cd ~/ros2_ws
colcon build --mixin release
```
- **Uwaga** budowanie może trwać ponad **20 min**. Proszę wykorzystać ten czas na zapoznanie się z informacjami, które w danym momencie podaje narzędzie `colcon`. W osobnym terminalu można uruchomić program `htop` i zaobserwować użycie pamięći RAM oraz rdzeni procesora podczas kompilacji.
5. Po zakończeni budowy, nalezy paczki wczytać do aktualnego kontekstu roboczego:
```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
LUB
```bash
source install/setup.bash
```

6. (**Opcjonalne**) Można przetestować działanie MoveIt z przykładowym ramieniem robotycznym *Franka*. W tym celu należy:
- naprawić plik `~/ros2_ws/src/moveit2_tutorials/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py`, poprzez dodanie linijki `rviz_node_tutorial,` w sekcji `nodes_to_start = [`.
- przebudowanie projektu komendami w punkcie 4.
- uruchomienie tutoriala:
```bash
ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true
```
- informacje o sterowaniu (w języku angielskim) znajdują się [w oficjalnej dokumentacji MoveIt 2](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)

---
# (OPCJONALNE TODO) Zmiana implementacji DDS
Zgodnie z informacją na tutorialu MoveIt 2, 26 wrzesnia 2022 wystepował problem z domyślną implementacją DDS w ROS 2 middleware. W celu jej obejścia, należało doinstalować CycloneDDS:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
i w każdym środowisku ustawiać zmienną środowiskoą (lub dodać ją do `~/.bashrc`):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


---
# Tematy i koncepcje do wytłyumaczenia
- Sterowanie przegubami RealTime (RTDE) oraz odpalanie programu (Dashboard)
- Kontrolery (ROS 2 Control)
- Planery trajektorii
- Roboty przemysłowe a koboty - różnice
- Obwody safety - GRZYBY MUSZA BYĆ SPRAWNE
- Inicjalizację robota - STOP, E-STOP, Booting, Running, itd.
---

# Krok 1 - Przygotowanie robota do zdalnego sterowania
**Poniższe kroki zakładają, że zarówno komputer i robot zostały już uruchomione**.


# Krok 2 Instalacja UR Robot Driver
Podobnie jak podczas instalacji MoveIt, należy sklonować repozytorium, pobrać brakujące zależności i zbudować paczkę sterownika robota UR.
```bash
git clone --branch humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
cd ~/ros2_ws
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

## 1. Konfiguracja sieci
Na stanowisku nr 1, komputer PC i robot UR powinny mieć następujące adresy IP:
- **PC**: `192.168.0.15`
- **UR3 CB**: `192.168.0.10`

W celu ich weryfikacji:
- **Na PC (Ubuntu)**, otworzyć terminal i uruchomić komendę `ip addr show`.
- **Na robocie (UR3 CB)**, w głównym menu PolyScope wybrać `Setup Robot`->`Network`, upewnić się, że zaznaczono opcję *DHCP*, a uzyskany adres zgadza się z w.w.

W pliku `/etc/hosts` można skonfigurować alias adresu IP (**wymagane są uprawnienia administratora**). Domyślnie, alias skonfigurowano jako `ur-robot`:
```bash
cat /etc/hosts
```

```
127.0.0.1		localhost
...
# ROBOLAB
192.168.0.10	ur-robot
...
```


**TODO**: Teraz konfiguracja Robota, sieci, pobranie kinematyki itd.

## 2. Kalibracja kinematyki robota
Na końcu linii produkcyjnej, każdy robot jest indywidualnie kalibrowany. Parametry kalibracji są niezbędne do przeprowadzania precyzyjnych obliczeń kinematyki prostej i odwrotnej. Plik ten jest zaszyty w kontrolerze robota. W celu obliczania prawidłowych nastaw przegubów z poziomu komputera, należy ten plik wyekstraktować. 

Aby nie zagłebiać się w dokumentację robota (i ręcznie implemnetować zapytania API do uzyskania tych danych), można wykorzystać gotowy skrypt paczki `ur_robot_driver`:
```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=ur-robot target_filename:="${HOME}/my_robot_calibration.yaml"
```
gdzie, argumenty:
- `robot_ip` - (string) jest adresem IP (lub aliasem) robota,
- `target_filename` - (string) jest ścieżką, pod którą ma zostać zapisany plik YAML z kalibracją robota.


## 3. Instalacja URCap
W celu wykorzystania możliwości robota UR3 CB do zdalnego sterowania, należy wgrać do jego systemu rozszerzenie (tzw. *URCap*) o nazwie "*externalcontrol-1.0.5.urcap*".

Na potrzeby labolatorium **krok ten został już wykonany**. Szczegołowa instrukcja jak powtórzyć tą operację znajduje się w [oficjalnej dokumentacji ur_robot_driver](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/install_urcap_cb3.html).

## 4. Konfiguracja programu robota
Program na robocie składa się zarówno z kolejnych poleceń (kolokwialnie: "kodu") oraz konfiguracji stanowiska, w którego skład wchodzą między innymi: definicje płaszczyzn ograniczających przestrzeń roboczą i konfiguracja zewnętrznych peryferii (np. chwytaka).

Konfiguracja programu "od zera" jest czasochłonną operacją, którą należy wykonać na początku instalacji robota na danym stanowisku. Najczęściej wykonują to osoby techniczne, odpowiedzialne m.in. za postument dla robota, klatkę bezpieczeńśtwa i ułożenie połączeń elektrycznych.

Mająć powyższe na uwadze, **dla potrzeb laboratorium zostanie wykorzystany wcześniej skonfigurowany program po stronie robota**:
- W głównym ekranie PolyScope wybrać **Program Robot**,
- Następnie **Load Program**,
- Wybrać plik *robolab_c3_13.urp*,
- Sprawdzić czy suma kontrolna (hash) konfiguracji bezpieczeństwa w **prawym górnym rogu** teach petanda jest równa: `7EE6`.
  - W przypadku rozbieżnośći, należy skonsultować się z prowadzącym.

Część "kodu" po stronie robota sprowadza się do wywołania polecenia "*External Control*".

---
# Krok 3 - Uruchomienie UR Robot Driver & MoveIt 2


## Pliki launch
Poniższa sekcja powstała na podstawie [dokumententacji ur_robot_driver](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html). Poniższe kroki zakładają tylko i wyłącznie użycie *prawdziwego* robota, jednakże trzeba zaznaczyć, że istnieje możliwość uruchomienia robota w symulacji.

W paczce *ur_robot_driver* znajdują się dwa pliki launch:
- `ur_control.launch.py` - który uruchamia pełną kontrolę nad robotem (kontroler, monitory stanu, sterownik przegubów, oraz dashboard do zarządzania uruchomionym programem),
- `ur_dashboard_client.launch.py` - uruchamia tylko dashboarda.

Powyższe pliki mogą przyjąć rózne argumenty uruchomieniowe. Ich wytłumaczenie i spis znajduje się w [dokumentacji ur_robot_driver](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html#launch-files).

## Własny plik launch
Na podstawie [dokumentacji ur_calibration](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/082dd2a90e0b2ac8b69d69bacf662806a7a572b9/ur_calibration/README.md).

Aby wykorzystać kalibrację, należy utworzyć własną paczkę z własnym plikiem launch. Należy zacząć od utworzenia własnej, nowej paczki. **UWAGA! Proszę skonfigurować własną NAZWĘ PACZKI!** 

```bash
export PACKAGE_NAME=NAZWA_GRUPY_ur_launch
cd ~/ros2_ws/src
ros2 pkg create $PACKAGE_NAME --build-type ament_cmake  --dependencies ur_client_library \ --description "Package containing calibrations and launch files for RoboLab's UR robot."
```

Nastepnie tworzymy szkielet folderów paczki.
```bash
mkdir -p $PACKAGE_NAME/etc
mkdir -p $PACKAGE_NAME/launch
echo 'install(DIRECTORY etc launch DESTINATION share/${PROJECT_NAME})' >> $PACKAGE_NAME/CMakeLists.txt
```

Do folderu `etc` należy przekopiować wcześniej wyekstraktowaną kalibrację robota:

```bash
cp ~/my_robot_calibration.yaml ~/ros2_ws/src/$PACKAGE_NAME/etc/ur3_calibration.yaml
```

Następnie, należy skopiować bazowy plik launch dla robota UR3 CB do nowej paczki:
```bash
cp $(ros2 pkg prefix ur_bringup)/share/ur_bringup/launch/ur_control.launch.py ur3.launch.py
```

Skopiowany plik (tj. `launch/ur3.launch.py`) należy wyedytować. Zostaną do niego wprowadzone nastepujące zmiany:
- wskazanie plik konfiguracji robota w paczce. **UWAGA! Proszę pamiętać o WŁASNEJ nazwie paczki!**
	```python
	# ...
	kinematics_params = PathJoinSubstitution(
			[FindPackageShare("NAZWA_PACZKI"), "etc", "", "ur3_calibration.yaml"]
		)
	# ...
	```
- domyślny argument modelu robota ustawić na `ur3`,
	```python
	# ...
	declared_arguments.append(
            DeclareLaunchArgument(
                "ur_type",
                description="Type/series of used UR robot.",
                choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
                default_value="ur3",
            )
        )
	# ...
	```
- domyślmny argument adresu IP ustawić na `ur-robot`,
	```python
	# ...
	declared_arguments.append(
            DeclareLaunchArgument(
                "robot_ip", 
                description="IP address by which the robot can be reached.",
                default_value="ur-robot",
            )
        )
	# ...
	```
- wyłączyć domyślne uruchomienie RViza,
	```python
	# ...
	declared_arguments.append(
           DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
        )
	# ...
	```

Następnie, należy paczkę zbudować i wczytać:
```bash
colcon build --packages-select $PACKAGE_NAME
source ./install/local_setup.bash
```


## Dodanie modelu chwytu kamery + chwytaka do modelu robota

## Dodanie płaszczyzn Workspace'a

## Dodanie transformacji

## Uruchomienie


### Plik launch domyślny (NIEZALECANE)
W jednym terminalu, należy uruchomić driver:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=ur-robot launch_rviz:=false
```

### Plik launch z kalibracją robota (ZALACANE)
 
W jednym terminalu, należy uruchomić driver:
```bash
ros2 launch $PACKAGE_NAME ur3.launch.py
```


### Dalsze uruchamianie
Na teach petandzie, należy kliknąć przycisk **Play**, aby uruchomić program. Można osiągnąć ten sam efekt, wołająć serwis *dasboard*:

```bash
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}
```

W drugim terminalu, należy uruchomić MoveIt:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

---
# Krok 1 - Instalacja MoveIt & UR Driver
Do poruszenia robotem UR z poziomu RViza będzie potrzebne:
- Paczka MoveIt z planerami ruchu i pluginem wizualizacyjnym do RViza
- Kontroler robota zgodny z ros_control - ros2_ur_driver
- Plugin do systemu operacyjnego robota (URcap) - external control.

1. Konfiguracja robota została już wykonana - urcap jest zainstalowany
1. Instalacja MoveIta
   1. 


WIP

ur_robot_driver [repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

ur_robot_driver [docs](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/index.html)

moveit 2 [docs](https://moveit.picknik.ai/humble/index.html)


# Odczyt IO z robota
Typ: `ur_msgs/msg/IOStates`
```bash
ros2 topic echo /io_and_status_controller/io_states
```

# Sterowanie narzędziem robota

Na podstawie [ur_msgs]():
```
# digital tool output
int8 PIN_TOOL_DOUT0 = 16
int8 PIN_TOOL_DOUT1 = 17

# Note: 'fun' is short for 'function' (ie: the function the service should perform).
int8 FUN_SET_DIGITAL_OUT = 1
int8 FUN_SET_FLAG = 2
int8 FUN_SET_ANALOG_OUT = 3
int8 FUN_SET_TOOL_VOLTAGE = 4

# valid values for 'state' when setting digital IO or flags
int8 STATE_OFF = 0
int8 STATE_ON = 1
```

Zasady  działania chwytaka pneumatycznego firmy Shmalz: 
- podanie `1` na Tool Input 1 uruchamia pompkę. `0` wyłącza pompkę.
- podanie `1` na Tool Input 0 wypuszcza powietrze. `0` zatyła zawór.


| Stan | Efekt       |
| ---- | ----------- |
| `00` | Brak chwytu |
| `10` | Upuszczenie |
| `11` | Upuszczenie |
| `01` | Chwyt       |

Chwyt
```bash
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 1}"
```

Opuszczenie
```bash
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 1}"
```



# Instalacja Intel RealSense D405
Kamerka głebi.
Oficjalne [repozytorium](https://github.com/IntelRealSense/realsense-ros#installation-instructions)
```bash
sudo apt install -y ros-humble-realsense2-camera
```
```bash
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
```

# Instalacja kamery Orbbec Astra
Kamerka głebi na USB 2.0.
Oficjalne [repozytorium](https://github.com/orbbec/ros_astra_camera/).
```bash
sudo docker pull husarion/astra:humble
```
```bash
sudo docker run --rm -it \
	--device /dev/bus/usb/ \
	husarion/astra:humble \
	ros2 launch astra_camera astra_mini.launch.py
```

## Poniższe REVERTnąć
```bash
source /opt/ros/humble/setup.bash
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
cd ~/repos
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/ros_astra_camera.git
cd ~/ros2_ws


```

# Krok X - Wyzerowanie systemu dla kolejnej grupy
- TODO
```bash
sudo apt remove -y ros-humble-desktop
```