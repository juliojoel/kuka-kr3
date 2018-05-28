clear all
close all
clc

%Posi��o da ferramenta
T = [0 0 120.32];
 
%Inicializa��o do rob� com ferramenta
kr3Init(T)

%Acesso � variavel global criada em kr3init
global kr3

%Defini��o dos �ngulos desejados
Q = [10 -100 111 166 100 333]

%C�lculo da cinem�tica direta
matDireta = cineD(Q)

%C�lculo da cinem�tica inversa
Qn = cineI(matDireta)

teach(Q)           %Visualiza �ngulos iniciais
%teach(Qn(1,:))     %Visualiza �ngulos calculados(primeira solu��o)