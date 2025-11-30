import numpy as np
import matplotlib.pyplot as plt
from IPython import display
import matplotlib.animation as ani
class Dinamički_bicikl: #klasa cijelog vozila koja pamti karakteristike vozila, trenutne kinematičke veličine i ima svoje metode
    def __init__ (self,m,Cr,Cf,lr,lf,Iz,vlon,vlat,psidot,steer): #osnovni parametri i liste/polja
        self.m=m #masa vozila
        self.Cr=Cr #koeficijent lateralnog trenja stražnjeg kola
        self.Cf=Cf #koeficijent lateralnog trenja prednjeg kola
        self.lr=lr #duljina wheelbasea iza centra mase
        self.lf=lf #duljina wheelbasea ispred centra mase
        self.Iz=Iz #moment inercije vozila 
        self.vlon=vlon #longitudalna brzina- konstantna u zavoju
        self.vlonlist=[vlon]
        self.vlat=[vlat] #matrica lateralnih brzina 
        self.psidot=[psidot] #matrica yaw ratea tojest kutnih brzina
        self.alat=[0] #matrica lateralnih ubrzanja/akceleracija
        self.omegadot=[0] #matrica kutne akceleracije/yaw akceleracije
        self.steer=steer #steer angle
        self.xlat=[0] #matrica lateralnih pomaka
        self.psi=[0] #matrica kutnih pomaka
        self.alphar=[0] #matrica slip anglea stražnjeg kola
        self.alphaf=[0] #matrica sip anglea prednjeg kola
        self.oveersteer=[]
        self.t=[0]
        self.xlon=[0]
        self.x=[0]
        self.y=[0]
    def reset(self): #metoda koja resetira sve liste/polja/matrice
        self.alat=[0] 
        self.omegadot=[0]
        self.vlat=[self.vlat[0]]
        self.psidot=[self.psidot[0]]
        self.xlat=[0]
        self.psi=[0]
        self.alphar=[0]
        self.alphaf=[0]
        self.t=[0]
        self.xlon=[0]
        self.x=[0]
        self.y=[0]
    def move(self,dt): #metoda koja pomice diferencijalne jednadžbe za jedan korak
        self.vlonlist.append(self.vlon)
        self.alat.append((-self.Cr-self.Cf)/(self.m*self.vlon)*self.vlat[-1]+(((self.Cr-self.Cf)/(self.m*self.vlon))-self.vlon)*self.psidot[-1]+(self.Cf/self.m)*self.steer)
        self.omegadot.append(((self.lr*self.Cr-self.lf*self.Cf)/(self.Iz*self.vlon))*self.vlat[-1]+((-self.lf**2*self.Cf+self.lr**2*self.Cr)/(self.Iz*self.vlon))*self.psidot[-1]+(self.Cf/self.Iz)*self.steer)
        self.vlat.append(self.vlat[-1]+self.alat[-1]*dt)
        self.psidot.append(self.psidot[-1]+self.omegadot[-1]*dt)
        self.xlat.append(self.xlat[-1]+self.vlat[-1]*dt)
        self.xlon.append(self.xlon[-1]+self.vlon*dt)
        self.psi.append(self.psi[-1]+self.psidot[-1]*dt)
        self.x.append(self.xlon[-1]*np.cos(self.psi[-1])+self.xlat[-1]*np.cos(self.psi[-1]+np.pi/2))
        self.y.append(self.xlon[-1]*np.sin(self.psi[-1])+self.xlat[-1]*np.sin(self.psi[-1]+np.pi/2))


        self.alphar.append((self.vlat[-1]-(self.psidot[-1]*self.lr))/(self.vlon))
        self.alphaf.append((-self.vlon*self.steer)/(self.vlon+(self.vlat[-1]+self.psidot[-1]*self.lf)*self.steer))
        self.t.append(self.t[-1]+dt)
        
    def test(self,t,dt): #testiranje za n koraka
        self.reset()
        for i in np.arange(0,t,dt):
            self.move(dt)
        print(self.alat)
        print(self.omegadot)
        print(self.vlat)
        print(self.psidot)
        print(self.xlat)
        print(self.psi)
    def oversteertest(self): #stvaranje matrice koja testira za oversteer/understeer
        for i in range(len(self.alphar)):
            ackermannkin=self.steer #kinetički ackermann kut je jednak steer angleu
            ackermanndin=self.steer-self.alphaf[i]+self.alphar[i] # dinamički ackermann kut
            if ackermannkin>ackermanndin: # testiranje za oversteer
                self.oveersteer.append("Oversteer")
            elif ackermannkin<ackermanndin: #testiranje za understeer
                self.oveersteer.append("Understeer")
            else:
                self.oveersteer.append("Neutral steering")
        print(self.oveersteer)
    def graph(self,t,dt,colour="black"):
        self.reset()
        for i in np.arange(0,t,dt):
            self.move(dt)
        plt.plot(self.x,self.y,color=colour)
        plt.savefig("graph.png")
        plt.show()
    def animation(self,save=True):
        plt.rcParams["figure.figsize"] = [5, 5]
        plt.rcParams["figure.autolayout"] = True
        fig=plt.figure()
        ax=plt.axes(xlim=(-2000,2000),ylim=(-2000,2000))
        
        line,=ax.plot([],[],color="black")
        def init():
            line.set_data([],[])
            return line,
        def animate(i):
            x1=self.x[:i]
            y1=self.y[:i]
            line.set_data(x1,y1)
            return line,
        anim = ani.FuncAnimation(fig, animate, init_func=init, frames=1000, interval=30, blit=False)
   
        if save:
            writer=ani.FFMpegWriter(fps=30)
            anim.save("test1.gif",writer=writer)
