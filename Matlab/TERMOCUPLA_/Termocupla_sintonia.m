%%
clc
clear all
close all
%% Cargar Datos
Dato=load('TermoData.mat');
Temp=Dato.Temp;
plot(Temp)
xlabel('Muestras, ts=100[ms]');
ylabel('Temperatura [°C]');
grid on;