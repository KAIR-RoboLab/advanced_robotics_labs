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

Do folderu `src` sklonuj repozytorium [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) z najnowszym stabilnym tagiem (w dniu pisania instrukcji jest to tag 4.1.1). Następnie stwórz nową paczkę ROSową. W tej paczce w pliku`package.xml` dodaj jako dependency **behaviortree_cpp**. Tworzymy nowy node C++ i dodajemy w pliku `CMakeLists.txt` jako zależność do budowania `behaviortree_cpp`.

Następnie postępując zgodnie z instrukcjami na stronie [BehaviorTree tutorial](https://www.behaviortree.dev/docs/category/learn-the-basic-concepts) (Wystarczy przeczytać od początku do [02 Blackboard and ports](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_ports)) należy stworzyć proste drzewo wyświetlające tekst na standardowym wyjściu terminala. Proszę dać chociaż jeden liść czekający na wejście od użytkownika i na jego bazie przynajmniej zwracać statusy `SUCCESS` i `FAILURE`. Czekanie na wejście może się dziać w sposób blokujący (blokujące akcje w drzewach bechawioralnych nie są wskazane). Mile widziane wykorzystanie portów, ale nie jest ono konieczne.

Mając kilka liści napisanych w C++ proszę za pomocą programu [Groot 2](https://www.behaviortree.dev/groot) skomponować drzewo.

> **Info** Warto w kodzie dodać `BT::StdCoutLogger logger_cout(my_tree);`, pozwoli to na podgląd stanu drzwea w czasie rzeczywistym.

## Zad 3

Na bazie dostarczonej w tym repozytorum paczki ROSowej należy Stworzyć drzewo nawigujące robotem do celu.
Napisane są już liście pozwalające na:
- `nav_to_goal_node::NavToGoalNode`: Nawiguje robota do celu wskazanego przez blackboard.
- `btn_state_node::BTNStateNode`: Sprawdza stan przyciusku na topicu wskazanym przez parametr z blackboardu `topic_name`.

Należy skomponować dowolne zachowanie robota z wykorzystaniem drzewa bechawioralnego.
