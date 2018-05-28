%    kr3Teach - Teach pendant gr�fico
%
%    A fun��o permite que o usu�rio manipule o rob� utilizando uma
%    interface gr�fica
%
%    Uso: kr3Teach(q)
%
%    Argumentos:
% 	      q = �ngulos das juntas em graus (Denavit-Hartemberg)
%                       * par�metro opcional, padr�o [0 -90 90 80 0 0]
%
%    A��o:
%           Cria figura para manipula��o do rob�
function kr3Teach(q)

global kr3 Qd

%Verifica se o par�metro foi informado
if ~exist('q','var')
    q=Qd;
end

qrad = q*pi/180;

%Verifica se �ngulos est�o dentro dos limites das juntas
L = kr3.links;
if (qrad(1) < L(1).qlim(1)) || (qrad(1) > L(1).qlim(2))
    error('�ngulo da junta 1 excede os limites.');
elseif (qrad(2) < L(2).qlim(1)) || (qrad(2) > L(2).qlim(2))
    error('�ngulo da junta 2 excede os limites.');
elseif (qrad(3) < L(3).qlim(1)) || (qrad(3) > L(3).qlim(2))
    error('�ngulo da junta 3 excede os limites.');
elseif (qrad(4) < L(4).qlim(1)) || (qrad(4) > L(4).qlim(2))
    error('�ngulo da junta 4 excede os limites.');
elseif (qrad(5) < L(5).qlim(1)) || (qrad(5) > L(5).qlim(2))
    error('�ngulo da junta 5 excede os limites.');
elseif (qrad(6) < L(6).qlim(1)) || (qrad(6) > L(6).qlim(2))
    error('�ngulo da junta 6 excede os limites.');
end

%Desabilita warning
warning('off','RTB:SerialLink:plot');

%Inicializa teach pendant gr�fico
kr3.teach(qrad, 'notiles', 'floorlevel', 1, 'lightpos', [0 0 -20])

%Configura visualiza��o da figura
set(gca, 'ZDir', 'reverse', 'YDir', 'reverse', ...
    'ZLim', [-1400 1], 'YLim', [-700 700], 'XLim', [-700 700], ...
    'ZLimMode', 'manual', 'YLimMode', 'manual', 'XLimMode', 'manual');