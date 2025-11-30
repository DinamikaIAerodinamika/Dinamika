clc;clear;close all;
C.m=300; %masa vozila
C.l=1.530; %duljina wheelbasea
C.lr=0.6885; % duljina stražnjeg dijela vozila
C.lf=0.8415; % duljina prednjeg dijela vozila
C.Iz=100; % moment inercije
C.Cr=1.4; % koeficijent trenja stražnje gume
C.Cf=1.4; %koeficijent trenja prednje gume
C.vlon=10; %longitudinalna brzina
C.vlat0=0.1; % početna lateralna brzina
C.psidot0=0.1; % početna kutna brzina
C.steer=0.23; % steering angle
function Rez=move_const(C,t,dt,steer) %funkcija koja  pomiče vozilo za t, koracima od dt
         Rez.alat=zeros(int16(t/dt),1); %početna lista lateralnih akceleracija
         Rez.omegadot=zeros(int16(t/dt),1);%početna lista kutnih akceleracija
         Rez.vlat=zeros(int16(t/dt),1); %početna lista lateralnih brzina
         Rez.psidot=zeros(int16(t/dt),1); % početna lsita kutnih brzina
         Rez.xlat=zeros(int16(t/dt),1); %početna lista lateralnih pomaka
         Rez.psi=zeros(int16(t/dt),1); %početna lista yaw anglea
         Rez.alphar=zeros(int16(t/dt),1); %početna lista slip anglea stražnjeg kola
         Rez.alphaf=zeros(int16(t/dt),1);%početna list slip anglea prednjeg kola
         Rez.oversteer=(zeros(int16(t/dt),1)); %početna lista over/understeera
         Rez.oversteerangle=(zeros(int16(t/dt),1)); %početna lista kutova oversteera
         Rez.t_lista=zeros(int16(t/dt),1); %početna lista vremena
         Rez.vlat(1)=C.vlat0; % prvi član liste lateralnih brzina
         Rez.psidot(1)=C.psidot0; %prvi član liste kutnih brzina
         Rez.oversteer(1)=0; %prvi član liste  over/understeera
         Rez.t_lista(1)=dt; %prvi član liste vremena
         Rez.oversteerangle(1)=0;%prvi član liste kutova oversteera

       
        for i=2:int16(t/dt) %rješavanje diferencijalne jednadžbe dinamičkog modela bicikla
             Rez.t_lista(i)=double(i)*dt; %i-ti član liste vremena
             Rez.alat(i)=((-C.Cr-C.Cf)/(C.m*C.vlon))*Rez.vlat(i-1)+(((C.Cr-C.Cf)/(C.m*C.vlon))-C.vlon)*Rez.psidot(i-1)+(C.Cf/C.m)*steer; %i-ti član liste lateralnih akceleracija
             Rez.omegadot(i)=((C.lr*C.Cr-C.lf*C.Cf)/(C.Iz*C.vlon))*Rez.vlat(i-1)+((-C.lf^2*C.Cf+C.lr^2*C.Cr)/(C.Iz*C.vlon))*Rez.psidot(i-1)+(C.Cf/C.Iz)*steer; %i-ti član liste kutnih akceleracija
             Rez.vlat(i)=Rez.vlat(i-1)+Rez.alat(i)*dt; % i-ti član liste lateralnih brzina (prijašnja brzina plus akceleracija puta dt)
             Rez.psidot(i)=Rez.psidot(i-1)+Rez.omegadot(i)*dt;% i-ti član liste kutnih brzina (prijašnja kutna brzina plus kutna akceleracija puta dt)
             Rez.xlat(i)=Rez.xlat(i-1)+Rez.vlat(i)*dt; %i-ti član liste lateralnih pomaka (prijašnji pomak plus brzina puta dt)
             Rez.psi(i)=Rez.psi(i-1)+Rez.psidot(i)*dt; %i-ti član liste yaw anglea (prijašnji kut plus kutna brzina puta dt)
             Rez.alphar(i)=atan((Rez.vlat(i)-(Rez.psidot(i)*C.lr))/(C.vlon)); %i-ti član liste stražnjih slip anglea 
             Rez.alphaf(i)=atan((-C.vlon*steer)/(C.vlon+(Rez.vlat(i)+Rez.psidot(i)*C.lf)*steer)); %i-ti član liste prednjih slip anglea
             Rez.oversteerangle(i)=Rez.alphar(i)-Rez.alphaf(i);%i-ti član liste kutova oversteera

             if Rez.alphar(i)>Rez.alphaf(i) %ako je stražnji slip angle veci od prednjeg imamo oversteer
                 Rez.oversteer(i)=1;
             elseif Rez.alphar(i)<Rez.alphaf(i)%ako je prednji slip angle veci od straznjeg imamo understeer
                 Rez.oversteer(i)=-1;
             else
                 Rez.oversteer(i)=0; %inače, imamo neutral steer
             end
           
        end
     %plot koji pokazuje je li u oversteeru ili understeeru
        %scatter(Rez.t_lista,Rez.oversteer)
        %yticks([-1,0,1])
        %yticklabels({"Understeer", "Neutral Steer", "Oversteer"})
     %plot koji pokazuje koliko understeera/oversteera
        scatter(Rez.t_lista,Rez.oversteerangle)
end
%move_const(C,5,0.01,0.23)

function Rez=move_steerinterval(C,t,dt,steerinterval) %funkcija koja  pomiče vozilo za t, koracima od dt
         Rez.steer=linspace(steerinterval(1),steerinterval(2),int16(t/dt));%lista vrijednosti steer anglea
         Rez.alat=zeros(int16(t/dt),1); %početna lista lateralnih akceleracija
         Rez.omegadot=zeros(int16(t/dt),1);%početna lista kutnih akceleracija
         Rez.vlat=zeros(int16(t/dt),1); %početna lista lateralnih brzina
         Rez.psidot=zeros(int16(t/dt),1); % početna lsita kutnih brzina
         Rez.xlat=zeros(int16(t/dt),1); %početna lista lateralnih pomaka
         Rez.psi=zeros(int16(t/dt),1); %početna lista yaw anglea
         Rez.alphar=zeros(int16(t/dt),1); %početna lista slip anglea stražnjeg kola
         Rez.alphaf=zeros(int16(t/dt),1);%početna list slip anglea prednjeg kola
         Rez.oversteer=(zeros(int16(t/dt),1)); %početna lista over/understeera
         Rez.oversteerangle=(zeros(int16(t/dt),1)); %početna lista kutova oversteera
         Rez.t_lista=zeros(int16(t/dt),1); %početna lista vremena
         Rez.vlat(1)=C.vlat0; % prvi član liste lateralnih brzina
         Rez.psidot(1)=C.psidot0; %prvi član liste kutnih brzina
         Rez.oversteer(1)=0; %prvi član liste  over/understeera
         Rez.t_lista(1)=dt; %prvi član liste vremena
         Rez.oversteerangle(1)=0;%prvi član liste kutova oversteera
        for i=2:int16(t/dt) %rješavanje diferencijalne jednadžbe dinamičkog modela bicikla
             Rez.t_lista(i)=double(i)*dt; %i-ti član liste vremena
             Rez.alat(i)=((-C.Cr-C.Cf)/(C.m*C.vlon))*Rez.vlat(i-1)+(((C.Cr-C.Cf)/(C.m*C.vlon))-C.vlon)*Rez.psidot(i-1)+(C.Cf/C.m)*Rez.steer(i); %i-ti član liste lateralnih akceleracija
             Rez.omegadot(i)=((C.lr*C.Cr-C.lf*C.Cf)/(C.Iz*C.vlon))*Rez.vlat(i-1)+((-C.lf^2*C.Cf+C.lr^2*C.Cr)/(C.Iz*C.vlon))*Rez.psidot(i-1)+(C.Cf/C.Iz)*Rez.steer(i); %i-ti član liste kutnih akceleracija
             Rez.vlat(i)=Rez.vlat(i-1)+Rez.alat(i)*dt; % i-ti član liste lateralnih brzina (prijašnja brzina plus akceleracija puta dt)
             Rez.psidot(i)=Rez.psidot(i-1)+Rez.omegadot(i)*dt;% i-ti član liste kutnih brzina (prijašnja kutna brzina plus kutna akceleracija puta dt)
             Rez.xlat(i)=Rez.xlat(i-1)+Rez.vlat(i)*dt; %i-ti član liste lateralnih pomaka (prijašnji pomak plus brzina puta dt)
             Rez.psi(i)=Rez.psi(i-1)+Rez.psidot(i)*dt; %i-ti član liste yaw anglea (prijašnji kut plus kutna brzina puta dt)
             Rez.alphar(i)=atan((Rez.vlat(i)-(Rez.psidot(i)*C.lr))/(C.vlon)); %i-ti član liste stražnjih slip anglea 
             Rez.alphaf(i)=atan((-C.vlon*Rez.steer(i))/(C.vlon+(Rez.vlat(i)+Rez.psidot(i)*C.lf)*Rez.steer(i))); %i-ti član liste prednjih slip anglea
             Rez.oversteerangle(i)=Rez.alphar(i)-Rez.alphaf(i);%i-ti član liste kutova oversteera

             if Rez.alphar(i)>Rez.alphaf(i) %ako je stražnji slip angle veci od prednjeg imamo oversteer
                 Rez.oversteer(i)=1;
             elseif Rez.alphar(i)<Rez.alphaf(i)%ako je prednji slip angle veci od straznjeg imamo understeer
                 Rez.oversteer(i)=-1;
             else
                 Rez.oversteer(i)=0; %inače, imamo neutral steer
             end
        end
      %plot koji pokazuje je li u oversteeru ili understeeru
        %scatter(Rez.t_lista,Rez.oversteer)
        %yticks([-1,0,1])
        %yticklabels({"Understeer", "Neutral Steer", "Oversteer"})
     %plot koji pokazuje koliko understeera/oversteera
        scatter(Rez.t_lista,Rez.oversteerangle)
end
%move_steerinterval(C,5,0.01,[0.23,0.7])

function Rez=move_speedinterval(C,t,dt,vlon_interval) %funkcija koja  pomiče vozilo za t, koracima od dt
         Rez.vlon=linspace(vlon_interval(1),vlon_interval(2),int16(t/dt));%lista vrijednosti steer anglea
         Rez.alat=zeros(int16(t/dt),1); %početna lista lateralnih akceleracija
         Rez.omegadot=zeros(int16(t/dt),1);%početna lista kutnih akceleracija
         Rez.vlat=zeros(int16(t/dt),1); %početna lista lateralnih brzina
         Rez.psidot=zeros(int16(t/dt),1); % početna lsita kutnih brzina
         Rez.xlat=zeros(int16(t/dt),1); %početna lista lateralnih pomaka
         Rez.psi=zeros(int16(t/dt),1); %početna lista yaw anglea
         Rez.alphar=zeros(int16(t/dt),1); %početna lista slip anglea stražnjeg kola
         Rez.alphaf=zeros(int16(t/dt),1);%početna list slip anglea prednjeg kola
         Rez.oversteer=(zeros(int16(t/dt),1)); %početna lista over/understeera
         Rez.oversteerangle=(zeros(int16(t/dt),1)); %početna lista kutova oversteera
         Rez.t_lista=zeros(int16(t/dt),1); %početna lista vremena
         Rez.vlat(1)=C.vlat0; % prvi član liste lateralnih brzina
         Rez.psidot(1)=C.psidot0; %prvi član liste kutnih brzina
         Rez.oversteer(1)=0; %prvi član liste  over/understeera
         Rez.t_lista(1)=dt; %prvi član liste vremena
         Rez.oversteerangle(1)=0;%prvi član liste kutova oversteera
        for i=2:int16(t/dt) %rješavanje diferencijalne jednadžbe dinamičkog modela bicikla
             Rez.t_lista(i)=double(i)*dt; %i-ti član liste vremena
             Rez.alat(i)=((-C.Cr-C.Cf)/(C.m*Rez.vlon(i)))*Rez.vlat(i-1)+(((C.Cr-C.Cf)/(C.m*Rez.vlon(i)))-Rez.vlon(i))*Rez.psidot(i-1)+(C.Cf/C.m)*C.steer; %i-ti član liste lateralnih akceleracija
             Rez.omegadot(i)=((C.lr*C.Cr-C.lf*C.Cf)/(C.Iz*Rez.vlon(i)))*Rez.vlat(i-1)+((-C.lf^2*C.Cf+C.lr^2*C.Cr)/(C.Iz*Rez.vlon(i)))*Rez.psidot(i-1)+(C.Cf/C.Iz)*C.steer; %i-ti član liste kutnih akceleracija
             Rez.vlat(i)=Rez.vlat(i-1)+Rez.alat(i)*dt; % i-ti član liste lateralnih brzina (prijašnja brzina plus akceleracija puta dt)
             Rez.psidot(i)=Rez.psidot(i-1)+Rez.omegadot(i)*dt;% i-ti član liste kutnih brzina (prijašnja kutna brzina plus kutna akceleracija puta dt)
             Rez.xlat(i)=Rez.xlat(i-1)+Rez.vlat(i)*dt; %i-ti član liste lateralnih pomaka (prijašnji pomak plus brzina puta dt)
             Rez.psi(i)=Rez.psi(i-1)+Rez.psidot(i)*dt; %i-ti član liste yaw anglea (prijašnji kut plus kutna brzina puta dt)
             Rez.alphar(i)=atan((Rez.vlat(i)-(Rez.psidot(i)*C.lr))/(Rez.vlon(i))); %i-ti član liste stražnjih slip anglea 
             Rez.alphaf(i)=atan((-Rez.vlon(i)*C.steer)/(Rez.vlon(i)+(Rez.vlat(i)+Rez.psidot(i)*C.lf)*C.steer)); %i-ti član liste prednjih slip anglea
             Rez.oversteerangle(i)=Rez.alphar(i)-Rez.alphaf(i);%i-ti član liste kutova oversteera

             if Rez.alphar(i)>Rez.alphaf(i) %ako je stražnji slip angle veci od prednjeg imamo oversteer
                 Rez.oversteer(i)=1;
             elseif Rez.alphar(i)<Rez.alphaf(i)%ako je prednji slip angle veci od straznjeg imamo understeer
                 Rez.oversteer(i)=-1;
             else
                 Rez.oversteer(i)=0; %inače, imamo neutral steer
             end
           
        end
     %plot koji pokazuje je li u oversteeru ili understeeru
        %scatter(Rez.t_lista,Rez.oversteer)
        %yticks([-1,0,1])
        %yticklabels({"Understeer", "Neutral Steer", "Oversteer"})
     %plot koji pokazuje koliko understeera/oversteera
        scatter(Rez.t_lista,Rez.oversteerangle)
end
%move_speedinterval(C,5,0.01,[10,50])

function Rez=move_ramtest(C,dt,steer_lista)
         Rez.alat=0;
         Rez.omegadot=0;
         Rez.vlat=[C.vlat0]; % prvi član liste lateralnih brzina
         Rez.psidot=[C.psidot0]; %prvi član liste kutnih brzina
         Rez.xlat=0;
         Rez.psi=0;
         Rez.alphar=0;
         Rez.alphaf=0;
         Rez.oversteer=0; %prvi član liste  over/understeera
         Rez.t_lista=dt; %prvi član liste vremena
         Rez.oversteerangle=0;%prvi član liste kutova oversteera

        for j=2:2:(size(steer_lista,2))
            for i=2:int16(steer_lista(j)/dt) %rješavanje diferencijalne jednadžbe dinamičkog modela bicikla
             Rez.t_lista=[Rez.t_lista,Rez.t_lista(end)+dt]; %i-ti član liste vremena
             Rez.alat=[Rez.alat,((-C.Cr-C.Cf)/(C.m*C.vlon))*Rez.vlat(end)+(((C.Cr-C.Cf)/(C.m*C.vlon))-C.vlon)*Rez.psidot(end)+(C.Cf/C.m)*steer_lista(j-1)]; %i-ti član liste lateralnih akceleracija
             Rez.omegadot=[Rez.omegadot,((C.lr*C.Cr-C.lf*C.Cf)/(C.Iz*C.vlon))*Rez.vlat(end)+((-C.lf^2*C.Cf+C.lr^2*C.Cr)/(C.Iz*C.vlon))*Rez.psidot(end)+(C.Cf/C.Iz)*steer_lista(j-1)]; %i-ti član liste kutnih akceleracija
             Rez.vlat=[Rez.vlat,Rez.vlat(end)+Rez.alat(end)*dt]; % i-ti član liste lateralnih brzina (prijašnja brzina plus akceleracija puta dt)
             Rez.psidot=[Rez.psidot,Rez.psidot(end)+Rez.omegadot(end)*dt];% i-ti član liste kutnih brzina (prijašnja kutna brzina plus kutna akceleracija puta dt)
             Rez.xlat=[Rez.xlat,Rez.xlat(end)+Rez.vlat(end)*dt]; %i-ti član liste lateralnih pomaka (prijašnji pomak plus brzina puta dt)
             Rez.psi=[Rez.psi,Rez.psi(end)+Rez.psidot(end)*dt]; %i-ti član liste yaw anglea (prijašnji kut plus kutna brzina puta dt)
             Rez.alphar=[Rez.alphar,atan((Rez.vlat(end)-(Rez.psidot(end)*C.lr))/(C.vlon))]; %i-ti član liste stražnjih slip anglea 
             Rez.alphaf=[Rez.alphaf,atan((-C.vlon*steer_lista(j-1))/(C.vlon+(Rez.vlat(end)+Rez.psidot(end)*C.lf)*steer_lista(j-1)))]; %i-ti član liste prednjih slip anglea
             Rez.oversteerangle=[Rez.oversteerangle,Rez.alphar(end)-Rez.alphaf(end)];%i-ti član liste kutova oversteera

             if Rez.alphar(end)>Rez.alphaf(end) %ako je stražnji slip angle veci od prednjeg imamo oversteer
                 Rez.oversteer=[Rez.oversteer,1];
             elseif Rez.alphar(end)<Rez.alphaf(end)%ako je prednji slip angle veci od straznjeg imamo understeer
                 Rez.oversteer=[Rez.oversteer,-1];
             else
                 Rez.oversteer=[Rez.oversteer,0]; %inače, imamo neutral steer
             end
            end
        end
     %plot koji pokazuje je li u oversteeru ili understeeru
        %scatter(Rez.t_lista,Rez.oversteer)
        %yticks([-1,0,1])
        %yticklabels({"Understeer", "Neutral Steer", "Oversteer"})
     %plot koji pokazuje koliko understeera/oversteera
        scatter(Rez.t_lista,Rez.oversteerangle)
end
move_ramtest(C,0.01,[[0.23,4],[0.3,7],[0.45,6]])
