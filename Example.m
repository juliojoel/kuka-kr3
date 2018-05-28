clear all
close all
clc

%Posição da ferramenta
T = [0 0 120.32];
 
%Inicialização do robô com ferramenta
kr3Init(T)

%Acesso à variavel global criada em kr3init
global kr3

%Definição dos ângulos desejados
Q = [10 -100 111 166 100 333]

%Cálculo da cinemática direta
matFK = kr3FKine(Q)

%Cálculo da cinemática inversa
Qn = kr3IKine(matFK)

kr3Teach(Q)           %Visualiza ângulos iniciais
%kr3Teach(Qn(1,:))     %Visualiza ângulos calculados(primeira solução)