// logdata assessment

clc;
clear;

data = csvRead("LOGFILE.TXT", ';', '.');

SI_TIn = data(:,3);
SI_HumIn = data(:,4);
SI_TDewIn = data(:,5);
SI_TOut = data(:,6);
SI_HumOut = data(:,7);
SI_TDewOut = data(:,8);
FC_StFan = data(:,9);
FC_TiFanOnTot = data(:,10);

plot(SI_TIn,'b')
hold on;
//plot(SI_TIn,'r')
//plot(FC_StFan,'k')
