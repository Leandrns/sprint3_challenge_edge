# sprint3_challenge_edge

<h1 align="center">Edge Challenge</h1>
<h2 align="center">Entrega da disciplina de Edge Computing</h2>
<div align="center">

  ![alt text](logo_CLIRV.png)

</div>

### Tecnologias
![Arduino](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
<br><br>
Simulação no Wokwi [aqui](https://wokwi.com/projects/400696539916996609)

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

## Componentes
<h3>Foram usados os seguintes componentes disponíveis na plataforma Wokwi:</h3>

- <p>1 Arduino Mega 2560</p>
- <p>1 Led Bar</p>
- <p>1 Slide-Potentiometer</p>
- <p>1 Display OLED ssd 1306</p>
- <p>1 Half-Breadboard</p>
- <p>1 GPS Fake-breakout</p>
- <p>10 Resistores de 220Ω</p>
- <p>42 cabos Jumper</p>

<h3>Foram usados os seguintes arquivos:</h3>

- <p>main.ino</p>
- <p>diagram.json</p>
- <p>NMEA.cpp</p>
- <p>NMEA.h</p>
- <p>gps-fake.chip.c</p>


## Requisitos
-  É necessário acessar a plataforma Wokwi através do [link](https://wokwi.com/projects/400696539916996609) do projeto.
- <p>Não é preciso criar uma conta no Wokwi para fazer a simulação;<p>
- Wokiwi pode ser acessado em qualquer navegador.

- <p>É necessário instalar as seguintes bibliotecas na aba Project Libraries: Adafruit SSD1306 e
Adafruit GFX Library</p>


## Circuito e Instruções de Uso
<div align=center>
  <img src="https://github.com/Leandrns/edge-challenge-mahindra/assets/162998083/4aae6882-300f-42b8-9b38-ab16c1367fda" width="800px">
  <img src="https://github.com/Leandrns/edge-challenge-mahindra/assets/162998083/8f2d33e5-3b99-4e37-9c67-7c9ba860c427">
</div>

<br><p>
  1. Para simular o projeto, basta apertar no botão de play do Wokwi. <br>
  2. O nível da bateria está sendo simulado através do potenciômentro slider (à esquerda), para ver o funcionamento basta arrastar a barra para cima ou para baixo. A oscilação pode ser observada através da barra de leds, localizada acima do Arduino.
</p>
