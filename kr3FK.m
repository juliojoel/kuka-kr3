%    kr3FK - Calcula a cinem�tica direta do rob�
% 
%    A fun��o retorna a matriz de transforma��o homog�nea de cinem�tica
%    direta do rob�
%
%    Uso: T0f = kr3FK(angulosDH)
% 
%    Argumentos:
% 	      angulosDH = angulos das juntas em graus (Denavit-Hartemberg)
%                       * par�metro opcional, padr�o [0 -90 90 80 0 0]
%
%    Retorno:
%           T0f = matriz homog�nea da cinem�tica direta
function T0f = kr3FK(angulosDH)

global kr3 Qd

fprintf('Calculando cinem�tica direta\n');

%Verifica se o par�metro foi informado
if ~exist('angulosDH','var')
    angulosDH=Qd;
    fprintf('�ngulos n�o informados, definido padr�o [0 -90 90 80 0 0]\n');
end

L = kr3.links;
F = kr3.tool.T;

a = angulosDH*pi/180;
f = F(1:3,4);

%Verifica se �ngulos est�o dentro dos limites das juntas
if (a(1) < L(1).qlim(1)) || (a(1) > L(1).qlim(2))
    error('�ngulo da junta 1 excede os limites.');
elseif (a(2) < L(2).qlim(1)) || (a(2) > L(2).qlim(2))
    error('�ngulo da junta 2 excede os limites.');
elseif (a(3) < L(3).qlim(1)) || (a(3) > L(3).qlim(2))
    error('�ngulo da junta 3 excede os limites.');
elseif (a(4) < L(4).qlim(1)) || (a(4) > L(4).qlim(2))
    error('�ngulo da junta 4 excede os limites.');
elseif (a(5) < L(5).qlim(1)) || (a(5) > L(5).qlim(2))
    error('�ngulo da junta 5 excede os limites.');
elseif (a(6) < L(6).qlim(1)) || (a(6) > L(6).qlim(2))
    error('�ngulo da junta 6 excede os limites.');
end

%%Cinem�tica direta do rob�
A1 = rotz(a(1)+L(1).offset)*tran([L(1).a,0,L(1).d])*rotx(L(1).alpha);
A2 = rotz(a(2)+L(2).offset)*tran([L(2).a,0,L(2).d])*rotx(L(2).alpha);
A3 = rotz(a(3)+L(3).offset)*tran([L(3).a,0,L(3).d])*rotx(L(3).alpha);
A4 = rotz(a(4)+L(4).offset)*tran([L(4).a,0,L(4).d])*rotx(L(4).alpha);
A5 = rotz(a(5)+L(5).offset)*tran([L(5).a,0,L(5).d])*rotx(L(5).alpha);
A6 = rotz(a(6)+L(6).offset)*tran([L(6).a,0,L(6).d])*rotx(L(6).alpha);
%Matriz de transforma��o homog�nea da ferramenta
Af = tran([f(1),f(2),f(3)]);

%Retorno da matriz de transforma��o homog�nea
T0f = round(A1*A2*A3*A4*A5*A6*Af,4);

%%Fun��es auxiliares de rota��o e transla��o
    function H = rotz(alpha)
        
        H = [cos(alpha), -sin(alpha), 0, 0;
            sin(alpha),  cos(alpha), 0, 0;
            0,           0, 1, 0;
            0,           0, 0, 1];
        
        function H = rotx(gama)
            
            H = [1,         0,          0, 0;
                0, cos(gama), -sin(gama), 0;
                0, sin(gama),  cos(gama), 0;
                0,         0,          0, 1];
            
            function H = tran(vet)
                
                H = [1, 0,  0, vet(1,1);
                    0, 1,  0, vet(1,2);
                    0, 0,  1, vet(1,3);
                    0, 0,  0,        1];
