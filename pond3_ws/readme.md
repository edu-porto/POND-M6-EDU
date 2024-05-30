# Atividade ponderada - 3

## Objetivo 

Com base nas atividades desenvolvidas na última ponderada é preciso implementar um frontend que faça a conexão com o robô e implementar um sistema de transmissão de imagens.


## Atividades Desenvolvidas 




### Workspace 

Na figura abaixo é possível conferir o workspace desenvolvido de acordo com o padrão ROS2. 

   ![Workspace ](./assets/workspace.png)



### Controle do robô
O sistema de controle do robô permite que a partir da cli o usuário consiga interagir com funções que foram definidas para o robô. Nesse caso da ponderada o robô tem as seguintes funções e com controles.

- Mover para Frente : tecla *w*
- Mover para Direita : tecla *d* 
- Mover para Esquerda : tecla *a*
- Mover de Ré : tecla *s*
- Freiar : tecla *espaço*

   ![controle](./assets/movement.png)



## Como utilizar a solução 

Em primeiro lugar é preciso estar com o Ubuntu 22.04 e o pacote ROS instalado. Caso essa etapa não esteja feita, recomendo seguir o seguinte tutorial para configurar o ambiente de desenvolvimento **[tutorial instalação ROS2](https://rmnicola.github.io/m6-ec-encontros/E01/ros)**.Além disso é preciso instalar o Webots para que seja possível rodar a simulação. 

Só basta executar no terminal: 
  ```console
sudo apt install webots
  ``` 


Considerando que o usuário já está em um ambiente Linux com o pacote ROS já configurado para rodar o projeto é preciso seguir os seguintes passos. 

 1. No terminal é preciso acessar a pasta raiz do workspace 

    ```console
    cd pond3_ws/src
    ``` 

2. Após garantir que o terminal está dentro da raíz do backend é preciso ligar a venv e instalar os requirements.

    ```console python3 - venv venv
        source venv/bin/activate 
        python3 pip install -r requirements.txt
    ``` 

3. Com o setup correto é só rodar o backend  :


    ```console
	 python3 main.py
      ```

4. Agora só basta instalar o frontend e usufruir da solução: 

    ```console 
    cd frontend/pond3

    npm install

    npm run dev
    ```  
  Logo em seguida é possível acessar o local host e usar a interface. 

5. Em outro terminal é preciso rodar o webots. Esse comando permite o usuário simular o turtlebot e controlar pelo frontend. 


    ```console
    ros2 launch webots_ros2_turtlebot robot_launch.py
    ``` 



Caso existam dúvidas sobre como instalar é possível conferir no vídeo de funcionamento abaixo como é feito o processo para instalar e utilizar a solução. 


## Demonstração dos trabalhos realizados 

No vídeo abaixo é possível conferir como executar o projeto e o funcionamento do mesmo. 

*to-do*
<!-- [![Demonstração](https://img.youtube.com/vi/UkCUKUDYCZ0/0.jpg)](https://www.youtube.com/watch?v=UkCUKUDYCZ0) -->

