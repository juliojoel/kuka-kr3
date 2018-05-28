%    kr3IK - Retorna a cinem�tica inversa do rob�
%  
%    A fun��o calcula a cinem�tica inversa do rob� manipulador.
%    Uma matriz com �ngulos em graus (Denavit-Hartemberg) � retornada, onde
%    cada linha representa uma solu��o poss�vel.
%
%    Uso: angulosDH = kr3IK(mat)
%
%    Argumentos:
% 	        mat = matriz homog�nea 4 x 4
%
%    Retorno:
%           angulosDH = �ngulos das juntas em graus (Denavit-Hartemberg)
function angulosDH = kr3IK(mat)

global kr3

%Verifica se o par�metro foi informado
if ~exist('mat','var')
    error('Par�metro n�o informado!');
end

fprintf('Calculando cinem�tica inversa\n');

L = kr3.links;
F = kr3.tool.T;

%Geometria do rob� e ferramenta
d1 = L(1).d;
a1 = L(1).a;
a2 = abs(L(2).a);
a3 = abs(L(3).a);
d4 = abs(L(4).d);
d6 = abs(L(6).d);
f  = F(3,4);

o1 = L(1).offset;

%Matrizes de transla��o e rota��o entre origem 0 e ferramenta
tra = mat(1:3,4);
rot = mat(1:3,1:3);

%%Theta_1
%Dist�ncia do punho em rela��o � origem, usando a junta 6+ferramenta
p04 = tra-(d6+f)*rot(1:3,3);

%Calcula theta_1 atrav�s da proje��o do punho no plano x0-y0
t_1aux_1 = atan2(p04(1,1), -p04(2,1));
%Tr�s solu��es para theta_1
t_1aux = [t_1aux_1, t_1aux_1+pi, t_1aux_1-pi];

%Busca de solu��es poss�veis
p = 0;
for i1=1:3
    t_1 = t_1aux(i1);
    %Verifica se theta_1 est� dentro dos limites da junta
    if (t_1 > L(1).qlim(1)) && (t_1 < L(1).qlim(2))
        
        %Dist�ncia punho em rela��o � junta 2
        p01 = [a1*cos(t_1+o1) a1*sin(t_1+o1) d1]'; %conhecido da cinem�tica direta
        p14 = p04-p01;
        %Postura do punho em rela��o � junta 2
        r01 = [ cos(t_1 + pi/2),  0, -sin(t_1 + pi/2);
            sin(t_1 + pi/2),  0,  cos(t_1 + pi/2);
            0, -1,                   0];  %conhecido da cinem�tica direta
        r10 = r01';
        p14r = r10*p14;
        
        %%Theta_3
        %�ngulo alpha obtido do triangulo formado pelas juntas 3,4,5
        alpha = atan2(a3,d4);
        %Hipotenusa do triangulo
        l4 = sqrt(d4^2+a3^2);
        %�ngulo beta entre hipotenusa e elo 2
        beta = acos((-a2^2-l4^2+norm(p14)^2)/(-2*a2*l4));
        %Quatro solu��es para theta_3
        t_3aux_1 = pi-(beta-alpha);
        t_3aux_2 = pi-(beta+alpha);
        t_3aux = [t_3aux_1, t_3aux_2, -t_3aux_1, -t_3aux_2];
        
        %%Theta_2
        %C�lculo dos �ngulos gamma e omega
        gamma = atan2(-p14r(2), -p14r(1));
        omega = acos((-a2^2-norm(p14)^2+l4^2)/(-2*a2*norm(p14)));
        %Quatro solu��es para theta_2
        t_2aux_1 = (gamma-omega);
        t_2aux_2 = (gamma+omega);
        t_2aux = [t_2aux_1, t_2aux_2, -t_2aux_1, -t_2aux_2];
        
        for i3=1:4
            t_3 = t_3aux(i3);
            %Verifica se theta_3 est� dentro dos limites da junta
            if (t_3 > L(3).qlim(1)) && (t_3 < L(3).qlim(2))
                for i2=1:4
                    t_2 = t_2aux(i2);
                    %Verifica se theta_2 est� dentro dos limites da junta
                    if (t_2 > L(2).qlim(1)) && (t_2 < L(2).qlim(2))
                        
                        %%Verifica��o dos �ngulos theta_1, theta_2 e
                        %%theta_3 calculados
                        
                        %Cinem�tica direta at� o centro do punho esf�rico
                        T1 = rotz(t_1+L(1).offset)*tran([L(1).a,0,L(1).d])*rotx(L(1).alpha);
                        T2 = rotz(t_2+L(2).offset)*tran([L(2).a,0,L(2).d])*rotx(L(2).alpha);
                        T3 = rotz(t_3+L(3).offset)*tran([L(3).a,0,L(3).d])*rotx(L(3).alpha);
                        T4 = tran([L(4).a,0,L(4).d]);
                        T03 = T1*T2*T3*T4;
                        vet = T03(1:3,4);
                        
                        %Verifica se solu��o � v�lida
                        if round(vet,2) == round(p04,2)
                            %Adiciona solu��o encontrada em Qp
                            p = p+1;
                            if p==1
                                Qp = [t_1, t_2, t_3];
                            else
                                Qp = [Qp; t_1, t_2, t_3];
                            end
                        end
                    end
                end
            end
        end
    end
end

if p==0
    error('Nenhuma solu��o encontrada!');
end

%Para cada solu��o Qp, resolve �ngulos theta_4, theta_5 e theta_6
r = 0;
for ip=1:p
    t_1 = Qp(ip,1);
    t_2 = Qp(ip,2);
    t_3 = Qp(ip,3);
    
    %%Cinem�tica direta at� punho esf�rico
    T1 = rotz(t_1+L(1).offset)*tran([L(1).a,0,L(1).d])*rotx(L(1).alpha);
    T2 = rotz(t_2+L(2).offset)*tran([L(2).a,0,L(2).d])*rotx(L(2).alpha);
    T3 = rotz(t_3+L(3).offset)*tran([L(3).a,0,L(3).d])*rotx(L(3).alpha);
    T4 = rotz(L(4).offset); %compensa��o do offset de L4
    T03 = T1*T2*T3*T4;
    
    %Matriz de transforma��o homog�nea do punho at� efetuador final
    R03 = T03(1:3,1:3);
    R3t = R03\rot;
    
    %%Theta 5
    %Entre 0 e pi
    t_4 = atan2(-R3t(2,3),-R3t(1,3));
    t_5 = atan2(sqrt(R3t(2,3)^2+R3t(1,3)^2),-R3t(3,3));
    t_6 = atan2(R3t(3,2),R3t(3,1));
    
    %Verifica se �ngulos est�o dentro dos limites das juntas
    if (t_4 > L(4).qlim(1)) || (t_4 < L(4).qlim(2)) || ...
            (t_5 > L(5).qlim(1)) || (t_5 < L(5).qlim(2)) || ...
            (t_6 > L(6).qlim(1)) || (t_6 < L(6).qlim(2))
        %Adiciona solu��o encontrada em Qr e Qn
        if ip==1 && r==0
            r = r+1;
            Qr = [t_4 t_5 t_6];
            Qn = [Qp(ip,:) Qr];
        else
            r = r+1;
            Qr = [t_4 t_5 t_6];
            Qn = [Qn; Qp(ip,:) Qr];
        end
        
        %Entre -pi e 0
        t_4 = atan2(R3t(2,3),R3t(1,3));
        t_5 = atan2(-sqrt(R3t(2,3)^2+R3t(1,3)^2),-R3t(3,3));
        t_6 = atan2(-R3t(3,2),-R3t(3,1));
        
        %Verifica se �ngulos est�o dentro dos limites das juntas
        if (t_4 > L(4).qlim(1)) || (t_4 < L(4).qlim(2)) || ...
                (t_5 > L(5).qlim(1)) || (t_5 < L(5).qlim(2)) || ...
                (t_6 > L(6).qlim(1)) || (t_6 < L(6).qlim(2))
            %Adiciona solu��o encontrada em Qr e Qn
            if ip==1 && r==0
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qp(ip,:) Qr];
            else
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qn; Qp(ip,:) Qr];
            end
        end
    else
        
        %Somente entre -pi e 0
        t_4 = atan2(R3t(2,3),R3t(1,3));
        t_5 = atan2(-sqrt(R3t(2,3)^2+R3t(1,3)^2),-R3t(3,3));
        t_6 = atan2(-R3t(3,2),-R3t(3,1));
        
        %Verifica se �ngulos est�o dentro dos limites das juntas
        if (t_4 > L(4).qlim(1)) || (t_4 < L(4).qlim(2)) || ...
                (t_5 > L(5).qlim(1)) || (t_5 < L(5).qlim(2)) || ...
                (t_6 > L(6).qlim(1)) || (t_6 < L(6).qlim(2))
            %Adiciona solu��o encontrada em Qr e Qn
            if ip==1 && r==0
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qp(ip,:) Qr];
            else
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qn; Qp(ip,:) Qr];
            end
        end
    end
end

if r==0
    error('Nenhuma solu��o encontrada!');
end

%%Retorno das solu��es encontradas
fprintf('%2i solu��es encontradas!\n', r);
angulosDH = round(Qn*180/pi,4);

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