# Release Notes
SolverBot release notes.

## Pacotes adicionados

- imu_raw
- md49_base_controller
- orocos-bayesian-filtering
- robot_pose_ekf
- ros_bridge_suite
- rplidar_ros
- solver
    - adc_raw
    - laser_control
    - laser_raw
    - solver_traction
    - temperature_w1_raw
- teleop_twist_keyboard

:file_folder: Os pacotes contem arquivos de launch para facilitar a execução de nós e uma pasta contendo um arquivo config para determinação de parâmetros importantes dos mesmos.

:exclamation: Os tópicos inscritos e publicados estão no formato:
- (Tipo da mensagem) Tópico

## Pacotes removidos

## Pacotes atualizados/modificados

### adc_raw
O pacote realiza a aquisição dos canais analógicos lidos através do módulo [MCP3208](http://ww1.microchip.com/downloads/en/DeviceDoc/21298e.pdf) e publica um vetor contendo os dados. 

Tópicos publicados:
- (adc_raw/Mcp3208_data) /adc_data

:warning: **Importante: Para que seja realizada a leitura em python, foi alterado na biblioteca do MCP3208 para a atual versão: spi bus = 0, device = 1. Caso a modificação não seja realiza, não será possíível realizar a leitura dos canais. Para a leitura em C++, é necessário a instalação da biblioteca WiringPi para comunicação SPI.** 

### laser_control
O pacote realiza o controle de velocidade com base na velocidade publicada pelo usuário, sendo esta publicada através da interface gráfica, e pelos obstáculos identificados pelo lasers. Além disso, publica a frequencia que os leds sejam acionados com base no os obstáculos identificados e na movimentação do robô. Dentro do mesmo pacote, é executado o nó que realiza o subscribe ao tópico de frequencia do led e realiza o controle sobre o mesmo.

Tópicos inscritos:
- (geometry_msgs/Twist) /solver_traction/cmd_vel
- (sensor_msgs/ChannelFloat32) /laser_front
- (sensor_msgs/ChannelFloat32) /laser_back
- (std_msgs/Float32) /led_frequency

Tópicos publicados:
- (geometry_msgs/Twist) /cmd_vel
- (std_msgs/Float32) /led_frequency

:exclamation: Caso as posições dos lasers estejam trocadas nas conecções físicas, é necessário alterar as posições no arquivo -> laser_control_defaults.yaml. Para fazer isto, basta executar o nó do pacote laser_raw e visuzalizar a posição do array correspondente ao laser vertical, sendo a primeira posição correspondente a 0.

:warning: **Importante: Para conseguir acessar a GPIO do nó led_control é necessário a instalação da biblioteca WiringPi.** 

### laser_raw
O pacote adicionado realiza a aquisição de dados dos valores analógicos lidos e calcula a distância do obstáculo identificado pelo sensor GP2Y0A21YK com base no [manual do fabricante](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf).

O pacote publica dois tópicos de formato *ChannelFloat32()* distintos, um relativo aos lasers frontais e outro dos lases traseiros. Os valores estão dispostos em um vetor (distancia0,distancia1,distancia2), sendo duas posições referentes aos lasers horizontais e uma referente ao laser vertical.

Tópicos inscritos:
- (adc_raw/Mcp3208_data) /adc_data

Tópicos publicados:
- (sensor_msgs/ChannelFloat32) /laser_front
- (sensor_msgs/ChannelFloat32) /laser_back

:warning: É necessário que as conecções físicas dos lasers estejam em sequência nas entradas analógicas para que a mensagem gerada contenha os três lasers em sequência, tanto frontais quanto traseiros.

### md49_base_controller
O comportamento da forma que o driver interpreta o comando de velocidade foi alterado para que o robô tenha mais mobilidade. A modificação foi realizada no caminho:
>/home/pi/catkin_ws/src/md49_base_controller/src/md49_base_controller/include/md49_base_controller/md49_base_controller_class.h

É possivel acessar outros dados ou modificar parametros visualizando a [documentação do driver](https://www.robot-electronics.co.uk/htm/md49tech.htm).

Tópicos inscritos:
- (geometry_msgs/Twist) /cmd_vel

Tópicos publicados:
- (md49_messages/md49_data) /md49_data
- (md49_messages/md49_encoders) /md49_encoders

Mais informações do pacote podem ser encontradas em:
>[wiki.ros.org/md49_base_controller](wiki.ros.org/md49_base_controller)

### orocos-bayesian-filtering
Pacote contendo a biblioteca Bayesian Filtering Library (BFL). Mais informações disponíveis em:

>[http://wiki.ros.org/bfl](http://wiki.ros.org/bfl)

### robot_pose_ekf
Pacete utilizado para estimar o modelo 3D do robô. Informações adicionais do pacote e forma de download disponível em:
>[http://wiki.ros.org/robot_pose_ekf](http://wiki.ros.org/robot_pose_ekf)

### solver
Metapacote contendo pacotes personalizados do robô.

### solver_traction
Pacote contendo o .launch geral do robô e funções extras, assim como night_mode, e nós responsáveis pela odometria do robô.

##### Night Mode
Nó responsável por habilitar o painél para operação noturna.

Tópicos publicados:
- (std_msgs/Bool) /solver/night_mode

##### Odometria
O pacote contém um conjunto de nós responsáveis por realizar a odometria do robô.

Tópicos inscritos:
- (md49_messages/md49_encoders) /md49_encoders

Tópicos publicados:
- (std_msgs/Float64) /left_wheel_vel
- (std_msgs/Float64) /right_wheel_vel
- (geometry_msgs/Twist) /vel_raw

### teleop_twist_keyboard
Pacote de movimentação genérico para robôs reponsivos à variável "Twist". Pacote disponível em:
>[http://wiki.ros.org/teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)

### temperature_w1_raw

Pacote que realiza aquisição da temperatura do robô.

Tópicos publicados:
- (sensor_msgs/Temperature) /ic/temperature

## Acesso remoto

Para que o usuário consiga realizar o acesso remoto do robô, é necessário utilizar o terminal de comando um ambiente de desenvolvimento que permita comunicação ssh. Com isso, é necessário digitar o seguinte comando:

```    
    ssh ubuntu@10.10.10.10
```

E inserir a senha: 2solve2020

A comunicação com o robô é realizada através do módulo VONETS 5G e o usuário recebe um endereço na faixa:

> 10.10.10.100 <-> 10.10.10.200

## Web App
Foi desenvolvido uma interface gráfica completa para o robô utilizando a biblioteca React para que o usuário consiga realizar o controle do mesmo através de um joystick virtual e visualizar a imagem da câmera instalada no robô dentro da própria interface Além disso, a interface dispõe de informações extras para monitoramento do usuário, assim como nível de tensão da bateria e temperatura interna do robô.

Para ter acesso à interface gráfica, insira na barra de endereços ou clicar no endereço abaixo após realizar a conecção:
>[10.10.10.10:3000](http://10.10.10.10:3000)

## Depedências extras

### Ros Melodic - Rasberry Pi
O sistema ROS utilizado na Raspberry Pi é o ROS Melodic. Para istalar o sistema, é necessário seguir a documentaçao presente em:
>[http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi)

Com isso, é criado o ambiente ros_catkin_ws contendo as bibliotecas base do sistema ROS.

### Python 3
Foi projetado para que o sistema funcione com base em Python 3. A instalação do ambiente catkin_ws com suporte ao Python 3 através dos seguintes comandos no terminal:

``` 
    sudo apt-get install python3-catkin-pkg-modules
    sudo apt-get install python3-rospkg-modules
```

### catkin_ws com suporte à Python 3
 Foi configurado um ambiente catkin_ws com suporte ao Python 3 para a instalação dos pacotes utilizados. Informações acerca dainformação são encontradas em:
>[http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Habilitar GPIO
Foi embarcado no robô um sistema para realizar o chaveamento da alimentação da bateria do driver através de uma porta GPIO. Sendo assim, é possível habilitar ou desabilitar via software o driver de alimentação dos motores. Para isso, foi utilizada a porta:

*GPIO Driver= 17*

### GPIOs Adicionais
Há dipónível externamente duas portas GPIO para uso do usuário. Sendo estas e seu uso atual, respectivamente:

*GPIO LEDs= 5*
*GPIO Farol= 6*
