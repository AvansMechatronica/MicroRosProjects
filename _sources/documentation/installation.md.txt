# Software
Om de microROS projecten te programmnen heb je de volgende software nodig:
- [Visual Studio Code](https://code.visualstudio.com/) (IDE)
- [Platform IO extension](https://platformio.org/install/ide?install=vscode) (extension voor Visual Studio Code)

Volg de installatie instructies van [Platform IO](https://platformio.org/install/ide?install=vscode) om de extension te installeren in Visual Studio Code.


## microROS agent
Daarnaast dien je een microROS agent te installeren op je computer. De microROS agent zorgt voor de communicatie tussen de microROS devices (zoals de ESP32) en ROS2 op je computer. 

```bash
mkdir -p ~/microROS_agent_ws/src
cd ~/microROS_agent_ws/src

# Verkrijg de juiste ROS2 distributie
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

cd ..
# Build de microROS agent
colcon build --symlink-install
source install/setup.bash
echo "source ~/microROS_agent_ws/install/setup.bash" >> ~/.bashrc
```


