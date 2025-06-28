% delete(arduinoObj)
clc;
clear;
close all;


arduinoObj = serialport("COM4",115200);
configureTerminator(arduinoObj,"CR/LF");
flush(arduinoObj);
configureCallback(arduinoObj,"terminator",@read_callback_serialport);

