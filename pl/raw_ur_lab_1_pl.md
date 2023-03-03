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



# Krok X - Wyzerowanie systemu dla kolejnej grupy
- TODO
