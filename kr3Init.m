%    kr3Init - Inicializa��o do rob� KR3
% 
%    A fun��o inicializa os par�metros do rob� manipulador Kuka KR3 R540
%
%    Uso: kr3Init(f)
%
%    Argumentos:
% 	      f = vetor de posicionamento da ferramenta em mm
%               * par�metro opcional, padr�o [0 0 0]
%
%    A��o:
%           Cria uma vari�vel global com nome kr3 contendo os par�metros
function kr3Init(f)

clear global;
global kr3 Qd

%Verifica se o par�metro foi informado
if ~exist('f','var')
    F=[0 0 0];
    fprintf('Rob� KR3 inicializado com ferramenta [0.00 0.00 0.00]\n');
else
    F = f;
    fprintf('Rob� KR3 inicializado com ferramenta [%3.2f %3.2f %3.2f]\n', F(1), F(2), F(3));
end

%Par�metros de Denavit-Hartenberg
L1 = Link('d',-345, 'a',-20, 'alpha',-pi/2, 'offset',      pi/2, 'qlim',[-170*pi/180 170*pi/180]);
L2 = Link('d',   0, 'a',260, 'alpha',    0, 'offset',        pi, 'qlim',[-170*pi/180  50*pi/180]);
L3 = Link('d',   0, 'a', 20, 'alpha', pi/2, 'offset',     -pi/2, 'qlim',[-110*pi/180 155*pi/180]);
L4 = Link('d',-260, 'a',  0, 'alpha',-pi/2, 'offset',-80*pi/180, 'qlim',[-175*pi/180 175*pi/180]);
L5 = Link('d',   0, 'a',  0, 'alpha', pi/2, 'offset',         0, 'qlim',[-120*pi/180 120*pi/180]);
L6 = Link('d', -75, 'a',  0, 'alpha',   pi, 'offset',        pi, 'qlim',[-350*pi/180 350*pi/180]);
L = [L1;L2;L3;L4;L5;L6];

%Inicializa rob� com os par�metros DH
kr3 = SerialLink(L, 'manufacturer', 'Kuka', 'name', 'KR 3 R540', 'tool', F);

%�ngulo padr�o das juntas
Qd=[0 -90 90 80 0 0];