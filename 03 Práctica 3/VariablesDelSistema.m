%%  Variable Pendulo invertido
%Ejecutar este archivo previo a ejecutar
%las simulaciones de los modelos.
clear
clc
close all
g=-9.81;

M=0.5 + 0.1*7;      %+ el 4 digito del DNI*100gr ; Masa del carro
m=0.2;              %+ el 3 digito del DNI*10gr; masa del péndulo
b=0.1;              % N/m/s  Coeficiente de fricción 
L=0.5 + 0.1*8;      %+ el 5 digito del DNI*10cm;   Longitud del péndulo
I=0.006;            % Inercia del péndulo

s = tf('s');

%REGULACION DEL PENDULO
G = (m*L*s^2) / ( ((M+m)*s^2+ b*s) * ((I + m*L^2)*s^2 - m*g*L) - m^2*L^2*s^4);          %FDT del pendulo
impulse(G)

G1 = zpk(G);                 %Representar la funcion para ver los polos y ceros

wn = sqrt(8.624);            %Obtener la frec de los polos rapidos

Fmuestreo = (100*wn)/(2*pi); %100 veces porque necesitamos que el pendulo oscile alrededor de los 90º

T= 1 / Fmuestreo;            %Periodo de muestreo

G1z = c2d(G1, T , 'zoh');    %Discretizacion con retenedor de orden 0 de la funcion del pendulo

z = tf('z',T);
C = (1250)*(z - 0.8277)*(z - 0.9788) / (z*(z - 1));     %Controlador del pendulo 

[num,den] = tfdata(C);       %Extraer por separado numerador y denominador
numC = cell2mat(num);        %Pasar a vector el numerador
denC = cell2mat(den);        %Pasar a vector el denominador


%REGULACION DEL CARRITO
Gc = ((I + m*L^2)*s^2 - m*g*L) / ( ((M+m)*s^2 + b*s) * ((I + m*L^2)*s^2 - m*g*L) - m^2*L^2*s^4 );        %FDT del carrito

%IMPORTANTE!! La frecuencia del sistema es igual en ambos ejemplos, por eso
%no se calcula en este apartado

G1c = zpk(Gc);               %Representar la funcion para ver los polos y ceros

G1cz = c2d(G1c, T , 'zoh');  %Discretizacion con retenedor de orden 0 de la funcion del carrito
Cc = (100)*(z - 0.98)*(z + 0.99) / (z*(z + 1)*(z - 0.61));      %Controlador del carrito
%Cc = (100)*(z - 0.98) / (z*(z - 0.67));

[numcarrito,dencarrito] = tfdata(Cc);   %Extraer por separado numerador y denominador para el controlador del carrito
numCcarrito = cell2mat(numcarrito);     %Pasar a vector el numerador del carrito
denCcarrito = cell2mat(dencarrito);     %Pasar a vector el denominador del carrito