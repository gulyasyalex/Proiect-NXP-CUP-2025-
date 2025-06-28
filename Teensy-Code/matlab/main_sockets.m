clc;
clear;
close all;

server = tcpserver(6789,"ConnectionChangedFcn",@newClientCallBack)
server.UserData.lastFlushed = 1;
server.UserData.figureHandle = figure;
server.configureTerminator("CR/LF");
server.configureCallback("terminator", @read_callback_serialport);


