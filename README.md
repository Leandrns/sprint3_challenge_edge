<h1 align="center">Challenge - Tech Mahindra</h1>
<h2 align="center">Entrega da disciplina de Edge Computing</h2>
<div align="center">

  ![alt text](logo_CLIRV.png)

</div>

### Tecnologias
![Arduino](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Node-RED](https://img.shields.io/badge/Node--RED-%238F0000.svg?style=for-the-badge&logo=node-red&logoColor=white)
<br><br>
Simulação no Wokwi [aqui](https://wokwi.com/projects/409857214540221441)

## Integrantes:
<p>Caio Alexandre dos Santos - RM: 558460</p>
<p>Leandro do Nascimento Souza - RM: 558893</p>
<p>Italo Caliari Silva - RM: 554758</p>
<p>Rafael de Mônaco Maniezo - RM: 556079</p>
<p>Vinicios Rozas Pannuci de Paula Cont - RM: 555338</p>

## Sobre o projeto
A empresa Tech Mahindra, em parceria com a FIAP, lançou um desafio para os alunos do primeiro ano de Engenharia de Software, com o objetivo de criar um site interativo para popularizar a Fórmula E, uma categoria de corrida de carros elétricos.<br>
Para isso, o grupo CLIRV Technologies está desenvolvendo um site com uma interface rica em informações sobre a Fórmula E, incluindo notícias, curiosidades, estatísticas, regras, circuitos e um calendário de corridas. O diferencial do site será um sistema gamificado, o "HitRace Fantasy FE", onde os usuários podem montar suas equipes com base em um sistema de compra com moedas fictícias e ganhar pontos conforme o desempenho real dos pilotos, motores, equipes e técnicos escolhidos.

## Funcionalidades
<p>Neste protótipo, vamos demonstrar a coleta de dados dos carros de corrida em tempo real para nossa plataforma. Os dados incluem a quantidade de bateria, a velocidade dos carros e a posição dos pilotos na pista. </p>
<p>Esses dados são enviados para o Node-Red utilizando a funcionalidade de WiFi do ESP32 e o protocolo de comunicação MQTT, a partir de um broker aberto no site HiveMQ.</p>
<p>Com os dados no Node-Red, é utilizada a biblioteca 'dashboard' para construir uma visualização mais amigável dos dados.</p>

## Componentes
<h3>Foram usados os seguintes componentes disponíveis na plataforma Wokwi:</h3>

- <p>1 ESP32</p>
- <p>1 Slide-Potentiometer</p>
- <p>1 Display OLED ssd 1306</p>
- <p>1 Half-Breadboard</p>
- <p>1 GPS Fake-breakout</p>
- <p>10 Resistores de 220Ω</p>
- <p>36 cabos Jumper</p>

<h3>Foram usados os seguintes arquivos:</h3>

- <p>main.ino</p>
- <p>diagram.json</p>
- <p>NMEA.cpp</p>
- <p>NMEA.h</p>
- <p>gps-fake.chip.c</p>
- <p>fake-gps.chip.json</p>
- <p>libraries.txt</p>


## Requisitos
-  É necessário acessar a plataforma Wokwi através do [link](https://wokwi.com/projects/409857214540221441) do projeto;
- <p>Não é preciso criar uma conta no Wokwi para fazer a simulação;<p>
- Wokwi pode ser acessado em qualquer navegador;
- <p>É necessário instalar as seguintes bibliotecas na aba Project Libraries: <code>Adafruit SSD1306</code>, <code>Adafruit GFX Library</code>, <code>ArduinoJSON</code> e <code>PubSubClient.</code></p>
- O Node.js precisa estar instalado no seu sistema. Caso não tenha instalado, baixe [aqui](https://nodejs.org/pt).
- <p>Caso não tenha o Node-Red instalado, execute o seguinte comando no cmd: <code>npm install -g --unsafe-perm node-red</code>.</p>
- <p>No Node-Red, é necessário instalar a biblioteca <code>node-red-dashboard</code></p>


## Circuito e Instruções de Uso
<div align=center>
  <img src="arduino_foto_nova.PNG" width="800px">
  <img src="https://github.com/Leandrns/edge-challenge-mahindra/assets/162998083/8f2d33e5-3b99-4e37-9c67-7c9ba860c427">
</div>

<br><p>
  1. Acesse o [HiveMQ Websocket Client](https://www.hivemq.com/demos/websocket-client/) e clique em "Connect" para se conectar ao broker. <br>
  2. Abra seu Prompt de Comando (cmd) e digite o comando <code>node-red</code>.
  3. Para abrir a interface do Node-Red, pesquise por <code>http://127.0.0.1:1880/</code> no seu navegador.
  4. Importe o arquivo <i>flow.json</i>.
  5. Para abrir o dashboard, pesquise por <code>http://127.0.0.1:1880/ui</code> no seu navegador.
  6. Para simular o projeto, basta apertar no botão de play do Wokwi. <br>
  7. O nível da bateria está sendo simulado através do potenciômentro slider (à esquerda), para ver o funcionamento basta arrastar a barra para cima ou para baixo. A oscilação pode ser observada através da barra de leds, localizada acima do Arduino.
</p>

## Draft de Arquitetura
<br><br>
<div align=center>
  <img src="iot.PNG" width="800px">
  <img src="backend.PNG" width="800px">
  <img src="fronts.PNG" width="800px">
</div>

