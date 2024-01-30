# aruco_flights

Эксперименты с полетами, используя Gazebo.

Скачать образ виртуальной машины можно отсюда: https://clover.coex.tech/ru/simulation_vm.html

## Подготовка

 * 8 Gb
 * 4 CPU
 * добавить привод оптических дисков и вставить VBoxGuestAdditions.iso
 * загрузить ВМ, запустить терминал, перейти в директорию CD-диска, выполнить:

```bash
cd /media/clover/VBox_GAs_7.0.6
sudo ./VBoxLinuxAdditions.run
sudo apt install mc htop nano
```

 * выключить ВМ

## Залезть в настройки и поменять

 * Дисплей: VMSVGA, включить 3D ускорение
 * общий буфер обмена: двунаправленный

## Подготовка карты и мира

```bash
# вместо ручного редактирования файлов выполнить команды:
sed -i 's|arg name="aruco_map" default="false"|arg name="aruco_map" default="true"|g' /home/clover/catkin_ws/src/clover/clover/launch/aruco.launch
sed -i 's|arg name="aruco_vpe" default="false"|arg name="aruco_vpe" default="true"|g' /home/clover/catkin_ws/src/clover/clover/launch/aruco.launch
sed -i 's|arg name="length" default="0.22"|arg name="length" default="0.33"|g' /home/clover/catkin_ws/src/clover/clover/launch/aruco.launch
sed -i 's|arg name="map" default="map.txt"|arg name="map" default="guap.txt"|g' /home/clover/catkin_ws/src/clover/clover/launch/aruco.launch
sed -i 's|arg name="aruco" default="false"|arg name="aruco" default="true"|g' /home/clover/catkin_ws/src/clover/clover/launch/clover.launch
# для генерации карты использовать:
# для изменения размера карты поменять два числа после 0.33
rosrun aruco_pose genmap.py 0.33 4 5 1 1 0 -o guap.txt
# в результате появится файл:
find ./ -name 'guap.txt'
#./catkin_ws/src/clover/aruco_pose/map/guap.txt
# посмотреть содержимое файла:
cat ./catkin_ws/src/clover/aruco_pose/map/guap.txt
# создаём мир с помощью команды из сгенерированной карты:
rosrun clover_simulation aruco_gen --single-model --source-world=/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover.world /home/clover/catkin_ws/src/clover/aruco_pose/map/guap.txt > /home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/guap.world
# подключаем мир
sed -i 's|clover_aruco.world|guap.world|g' /home/clover/catkin_ws/src/clover/clover_simulation/launch/simulator.launch
```
  
# Чтобы подключиться к существующему миру...

Надо посмотреть файл `/home/clover/catkin_ws/src/clover/clover/launch/aruco.launch` и найти название `???.txt` файла в параметре `map`.

Также надо посмотреть `/home/clover/catkin_ws/src/clover/clover_simulation/launch/simulator.launch` и найти название `???.world` в параметре `world_name`.

После чего поменять `???.txt` и перезапустить команду `rosrun clover_simulation aruco_gen  --single-model --source-world=/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover.world /home/clover/catkin_ws/src/clover/aruco_pose/map/???.txt > /home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/???.world` с правильными названиями файлов.

# Проблема с кодировкой latin-1

Иногда, по непонятным причинам ни в Visual Studio Code, ни в терминале питон программа не запускается. Пишет ошибку с latin-1. Способ решения проблемы:

```bash
# установить русскую локаль
sudo locale-gen ru_RU
sudo locale-gen ru_RU.UTF-8
sudo update-locale

#  принудительно запустить питон с UTF-8 кодировкой
PYTHONIOENCODING=UTF-8 python3 file.py
```

# Установка QGroundControl

Скачиваем последний релиз `.AppImage` с этой ссылки : https://github.com/mavlink/qgroundcontrol/releases перемещаем в папку `~/bin` и выдаем права на запуск.

Перед запуском возможно потребуется выполнить: 

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager
id
```

Если команда `id` не показала сразу то, что пользователь входит в группу `dialout`, то потребуется завершить сеанс пользователя и залогиниться снова (или перезапустить компьютер). Также перед запуском можно проверить есть ли доступ к порту для работы программы: `stat /dev/ttyACM0` не должна показывать ошибку.

## Заметки по поводу использования QGroundControl

Возможна проблема : не удаётся запустить калибровку чего-нибудь (например калибровку ESC), появляется надпись "TRANSITION DENIED : SHUTDOWN to INIT". Скорее всего процедура калибровки выполняется после полной перезагрузки (выключения) полётного контроллера, например, надо не только отключить АКБ, он и отключить полностью всё питание, в т.ч. и по USB. После выключения питания, включения и запуска калибровки, ошибка не появляется и калибровка проходит.

Возможны проблема : при запуске программы она ругается "qt.network.ssl: QSslSocket: cannot resolve EVP_PKEY_base_id". Чтобы исправить проблему ей нужна библиотека libssl1.1.1. Для того чтобы узнать где ее взять, надо последовательно выполнить следующие команды:

```bash
# проверить, установлена ли библиотека, если да, то не трогать
dpkg -s libssl1.1

# определить откуда взялся пакет:
apt-cache show libssl3 | grep Filename
cat /etc/apt/sources.list | grep "^deb .* main"
# в браузере перейти по адресу, например
# http://ru.archive.ubuntu.com/ubuntu/pool/main/o/openssl/
# скачать библиотеку, например
# libssl1.1_1.1.1f-1ubuntu2.20_amd64.deb
# (не стоит перебирать разные библиотеки, некоторые могут повредить систему, внимательно проверяй зависимости)

# установка
sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2.20_amd64.deb
# проверить зависимости
dpkg -s libssl1.1 | grep -E "(Version)|(Depends)|(Breaks)"
```

После перезапуска QGroundControl она не должна ругаться на QSslSocket.
