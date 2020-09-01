close all
clear all
clc

%animation

mdl_puma560;

t=[0:0.03:1]';

q=jtraj(qz,qr,t);

p560.plot(q)

p560.teach()