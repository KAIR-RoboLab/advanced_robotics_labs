# Behaviour tree

Behaviour tree jest koncepcją zaczerpniętą ze świata gier komputerowych. W ostatnich kilku latach bardzo ciepło pomysł ten został przyjęty w świecie robotyki i można go znaleźć w takich paczkach jak [Navigation2](https://navigation.ros.org/) (z którego korzystamy w ramach tego laboratorium), czy [MoveIt2](https://moveit.ros.org/) (używany w laboratoium z robotem UR3). W robotyce króluje implementacja drzew bechawioralnych [BehaviorTree.CPP](https://www.behaviortree.dev/), którą to będziemy używać w tym laboratorium. W czasie pisania tej instrukcji jedyną stabilną wersją jest wersja v3.8, której to będziemy też używać.

BT (Behaviour tree) posiadają też program do wizualizacji o nazwie [Groot](https://github.com/BehaviorTree/Groot). Pozwala on w sposób graficzny konfigurować nasze drzewa.

## Główne koncepcje

BehaviorTree.CPP zawdzięcza swoją popularność głównie dzięki skalowalności, możliwości szybkiej rekonfiguracji w locie i szybkości działania dzięki implementacji w C++.

W celu zrozumienia ich działania warto przeczytać coś z poniższych linków:
- [BehaviorTree tutorial](https://www.behaviortree.dev/docs/category/learn-the-basic-concepts) (tutorial dla developerów)
- [A survey of Behavior Trees in robotics and AI](https://www.sciencedirect.com/science/article/pii/S0921889022000513) (ponieważ instrukcja do laboratorium nie może się obejś bez cytowania publikacji)
- [Behavior Trees in Robotics and AI](https://arxiv.org/abs/1709.00084) (dłuższa i cięższa publikacja na temat zastosowań w robotyce z punktu widzenia automatyka-teoretyka)

W skrócie najważniejszymi informacjami jest to, że BehaviorTree.CPP opiera się o pliki XML konfigurujące drzewo składające się z liści pisanych w C++.

# Cele ćwiczenia

## Zad 1

### Uruchamianie robota i systemu nawigacji

#### Włączenie robota

Na tylnym panelu przełącz włącznik od robota i włącz zasialnie.
Po chwili sprawdź czy robot jest włączony logując się do niego przez ssh:
``` bash
ssh husarion@192.168.0.30
```

Uruchom
``` bash
docker compose up
```

#### Uruchomienie nawigacji

Na komputerze pobierz repozytorium [rosbot-slam](https://github.com/Kotochleb/rosbot-slam) z brancha `Kotochleb-fix-config-for-lab`.
Następnie pobierz statyczną mapę sali z Upela i dodaj ją do folderu **maps** w folderze pobranego repozytorium.

Postaw robota na ziemi i na komputerze uruchom:
``` bash
SLAM_MODE=localization docker compose -f compose.pc.yaml up
```

To utruchmi cały stos nawigacyny wraz z lokalizacją na statycznie stworzonej mapie sali labolatoryjnej.

> **Warning**: Lidar robota ma problem z widzeniem nóg stolików, daltego będzie próbował jechać na skróty pod stolikiem stanowiska z robotem UR3. Dlatego też można otworzyć mapę w programie Gimp lub MS Paint i dorysować czwny prostokąt w miejscu istnienie stanowiska. Mimo, że żeczywiste wskazanie lidaru będzie pokazywało, że w tym obszarze przestrzeń jest otwarta, to planer globalny będzie grzecznie omijał ten obszar.

#### Lokalizowanie robota

Jak widać robot nie ma pojęcia początkowo gdzie na sali się znajduje. Wskazania lidaru nie pokrywają się z rzeczywistością.
W pierwszej kolejności wciśniej przycisk `Add`, następnie zakładkę `By topic` i wybierz topic `/amcl/PoseWithCovariance`.
Pod robotem pojawi się różowa elipsa, a przed nią żółty trójkąt. Oznaczają one niepewność estymacji. Elipsa oznacza położenie, a trójąt rotację (and **heading**). 
Wegług systemu lokalizacji AMCL (Adaptive Monte Carlo Localization) robot znajduje się z równym pradopodobieństem wewnątrz tej elipsu. W przypadku złego wskazania punktu startowego można łatwo stwierdzić, że jest to dalekie od prawdy. Elipsa ta będzie się zwiększać, bądź zmniejszać w zależności od dokładności estumacji. Jeśli system lokalizacji ma problem zlokalizować robota, to elipsa, jak i trójąt będą się rozszerzać. Jeśli robot jest w stanie dobrze go zlokalizować, całość będzie sukcesywnie maleć.

Analogicznie do `/amcl/PoseWithCovariance` dodaj topic `/local_costmap/costmap` w celu wizualizacji przeszkód które napotyka robot w czasie jazdy, a które nie są uwzględnione na statycznej mapie globalnej. W prawym panelu rozwiń parametry topicu mapy i zmień `Color Scheme` na `costmap`.
> **Note**: Dla zainteresowanych można też wyświetlić mapę `/global_costmap/costmap` która pokazuje mapę statyczną po przejściu przez warstwę `inflation` layer. Rozmywa ona piksele przeszkód definioując płynne przejście określające prawdopodobieństwo wystąpienia przeszkody.

W celu wskazania wstępnej estymacji położenia robota w RViz2 wybierz `2D Pose Estimate` z panelu górnego i zaznacz na mapie mniej więcej gdzie postawiłeś robota.
Następnie za pomocą `2D Pose Goal` każ robotowi przejechać kawałek tak żeby dobrze się zlokalizował. Robot powinien przejechać

#### Praca w domu w symulacji

Chwilowo symulacja jest poprawiana. W najbliższym czasie pojawi się możliwość wykonywania laboratorium w pełni zdalnie. Na ten moment trwają aktywne poprawki symulacji wymagające dodatkowych pull requestów po stronie firmy Husarion. Zaleca się obserwować aktywnie repozytorium [advanced_robotics_labs](https://github.com/KAIR-RoboLab/advanced_robotics_labs) gdzie będzie można znaleźć najświeższą wersję instrukcji.


## Zad 2

Stworzyć swoją paczkę ROSową, dodać jej do zależności `behaviortree_cpp_v3` i napisać node który za pomocą loggera ROSowego wyświetla informację i wchodzi w interakcję z użytkownikiem. Należy przetestować node'y typu:
- `sequence`
- `fallback`
- `parallel`

Oraz pracować z wizualizację całości w programie Groot.

Przydatne materiały:
- [BehaviorTree tutorial](https://www.behaviortree.dev/docs/category/learn-the-basic-concepts)
- [BehaviorTree.CPP/examples](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/master/examples)

## Zad 3
Zaprogramować robota żeby działał jak robot usługowy, typu kelner. Jego zadaniem jest przejechać z punktu **A** do punktu **B**, dokonać interakcję i przejechać dalej. W celu interakcji można wykorzystać przyciki znajdujące się z tyłu robota.

Przydatne materiały:
- [nav2_behavior_tree](https://github.com/ros-planning/navigation2/blob/main/nav2_behavior_tree/README.md)

