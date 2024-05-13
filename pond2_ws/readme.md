# Atividade ponderada - 2

## Objetivo 
O objetivo dessa atividade é interagir com o turtlebot atráves de uma CLI.

## Atividades Desenvolvidas 



### Workspace 



## Como utilizar a solução 

Em primeiro lugar é preciso estar com o Ubuntu 22.04 e o pacote ROS instalado. Caso essa etapa não esteja feita, recomendo seguir o seguinte tutorial para configurar o ambiente de desenvolvimento **[tutorial instalação ROS2](https://rmnicola.github.io/m6-ec-encontros/E01/ros)**.

Considerando que o usuário já está em um ambiente Linux com o pacote ROS já configurado para rodar o projeto é preciso seguir os seguintes passos. 


#### Criar a venv

    ```console
python3 -m venv ven
    ``` 

#### Ligar a venv 

    ```console
source venv/bin/activate
    ``` 

#### Instalar os requirements
    ```console
python3 -m venv ven
    ``` 



 1.   No terminal é preciso acessar a pasta raiz do workspace 

    ```console
    cd pond2_ws/src
    ``` 

2. Após garantir que o terminal está dentro da raíz do workspace é preciso construir o pacote. Continuando no terminal é executado o seguinte comando 

    ```console
    colcon build
    ``` 

3. Com o pacote construido só basta executar :


    ```console
	. install/setup.bash
      ``` 

4. Em outro terminal é preciso rodar o turtlesim com o webots. Esse comando permite o usuário simular o turtlebot. 


    ```console
	ros2 launch webots_ros2_turtlebot robot_launch.py
      ``` 

5. Agora só basta executar o pacote e conferir o resultado. No terminal antigo que está na raíz do workspace execute : 

    ```console 
    ros2 run cli pond_2 
    ``` 

Caso existam dúvidas sobre como instalar é possível conferir no vídeo de funcionamento abaixo como é feito o processo para instalar e utilizar a solução. 


## Demonstração dos trabalhos realizados 

No vídeo abaixo é possível conferir como executar o projeto e o funcionamento do mesmo. 

[![Demonstração](https://img.youtube.com/vi/iquhxcx8Fas/0.jpg)](www.google.com)

