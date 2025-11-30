clc;clear;close all;
C.m=300;
C.l=1530;
C.lr=0.6885;
C.lf=0.8415;
C.Iz=100;
C.Cr=1.4;
C.Cf=1.4;
C.vlon=10;
C.vlat0=0.1;
C.psidot0=0.1;
C.steer=0.23;
function Rez=move(C,t,dt)
         Rez.alat=zeros(int16(t/dt),1);
         Rez.omegadot=zeros(int16(t/dt),1);
         Rez.vlat=zeros(int16(t/dt),1);
         Rez.psidot=zeros(int16(t/dt),1);
         Rez.xlat=zeros(int16(t/dt),1);
         Rez.psi=zeros(int16(t/dt),1);
         Rez.alphar=zeros(int16(t/dt),1);
         Rez.alphaf=zeros(int16(t/dt),1);
         Rez.vlat(1)=C.vlat0;
         Rez.psidot(1)=C.psidot0;
         %for i=2:int16(t/dt)
             %alat(i)=((-C.Cr-C.Cf)/(C.m/*C.vlon))*vlat(i-1)+((C.Cr-C.Cf)/(C.m*C.vlon))
         %end
end
Rez=move(C,5,0.333);
disp(Rez.psidot)
