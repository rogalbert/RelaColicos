clear all, close all, clc
currentFolder = pwd;
cd ..
try
    addpath(genpath([pwd '/robotarium-matlab-simulator']))
catch
    error("No tienes la carpeta del simulador")
end
cd(currentFolder);