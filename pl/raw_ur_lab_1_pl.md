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
---

# Krok 1 - Przygotowanie robota do zdalnego sterowania
**Poniższe kroki zakładają, że zarówno komputer i robot zostały już uruchomione**

## 1. Konfiguracja sieci
Na stanowisku nr 1, komputer PC i robot UR powinny mieć następujące adresy IP:
- **PC**: `192.168.1.15`
- **UR3 CB**: `192.168.1.10`

W celu ich weryfikacji:
- **Na PC (Ubuntu)**, otworzyć terminal i uruchomić komendę `ip addr show`.
- **Na robocie (UR3 CB)**, w głównym menu PolyScope wybrać `Setup Robot`->`Network`, upewnić się, że zaznaczono opcję *DHCP*, a uzyskany adres zgadza się z w.w.

W pliku `/etc/hosts` można skonfigurować alias adresu IP (**wymagane są uprawnienia administratora**). Domyślnie, alias skonfigurowano jako `ur-robot`:
```bash
cat /etc/hosts
# ------------------------------- #
127.0.0.1	localhost
...
# ROBOLAB
192.168.1.10	ur-robot
...
```

## 2. Kalibracja kinematyki robota
# TODO: PRZENIEŚĆ TO PO KOMPILACJI ŹRÓDŁA
Na końcu linii produkcyjnej, każdy robot jest indywidualnie kalibrowany. Parametry kalibracji są niezbędne do przeprowadzania precyzyjnych obliczeń kinematyki prostej i odwrotnej. Plik ten jest zaszyty w kontrolerze robota. W celu obliczania prawidłowych nastaw przegubów z poziomu komputera, należy ten plik wyekstraktować. 

Aby nie zagłebiać się w dokumentację robota (i ręcznie implemnetować zapytania API do uzyskania tych danych), można wykorzystać gotowy skrypt paczki `ur_robot_driver`:
```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=ur-robot target_filename:="${HOME}/my_robot_calibration.yaml"
```
gdzie, argumenty:
- `robot_ip` - (string) jest adresem IP (lub aliasem) robota,
- `target_filename` - (string) jest ścieżką, pod którą ma zostać zapisany plik YAML z kalibracją robota.

## 3. Instalacja URCap
W celu wykorzystania możliwości robota UR do zdalnego sterowania.

---
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

**TODO**: Teraz konfiguracja Robota, sieci, pobranie kinematyki itd.

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



# Krok X - Wyzerowanie systemu dla kolejnej grupy
- TODO
```bash
sudo apt remove -y ros-humble-desktop
```