import turtle
from math import *


curva = 180

# Dados do Cavalo
lfrontal = 2.49 #   Largura frontal
eixof = 0.91 # Recuo do eixo em relação a frente do veículo
d_eixo = 4 # Distância entre eixos
ltraseira = 2.49 #   Largura traseira
eixot = 0.91 # Recuo do eixo em relação a frente do veículo



alfa = 25 #ângulo máximo de esterçamento : alfa (Graus)
dx = 0.5 #Discretização, indicando o acrescimo de posição a cada iteração


t = turtle.Turtle()

wn = turtle.Screen()

wn.setworldcoordinates(-10,-5, 30,35)


# Eixos e extremidades do cavalo

t2 = t.clone() # Roda dianteira direita
t2.up()
t2.goto(eixof,lfrontal/2)


t3 = t.clone() # Roda dianteira esquerda
t3.up()
t3.goto(eixof,-lfrontal/2)


t4 = t.clone() # Eixo traseiro
t4.up()
t4.goto(-d_eixo,0)
d14 = -t4.xcor() # Distância entre t1(t) e t4

t5 = t.clone() # Lateral traseira direita
t5.up()
t5.goto(-d_eixo-eixot,-ltraseira/2)


t6 = t.clone() # Lateral traseira esquerda
t6.up()
t6.goto(-d_eixo-eixot,ltraseira/2)


colors = ["black","green","green","black","red","red"]
c = 0


for k in [t,t2,t3,t4,t5,t6]:
    k.color(colors[c])
    c = c + 1
    k.speed("fastest")
    k.down()


for _ in range(10):
    for k in [t,t2,t3,t4,t5,t6]:
        k.fd(dx)



alfa2 = 0
k = 0
a = 92
b = a

def tract0(t,t4):
    
    xc, yc = t.pos()
    
    x_t4,y_t4 = t4.pos()

    if (xc - x_t4) != 0:
        
        a_rad = atan((yc - y_t4)/(xc - x_t4))
        a = degrees(a_rad)

        if abs(t.heading() - a) < abs(t.heading() -180 - a):
            a_rad = a_rad         
        else:
            a_rad =  pi + a_rad
            
    else:
        a_rad = pi/2
        a = degrees(a_rad)
    
    a = degrees(a_rad)
    t4.setheading(a)

    dx4 = t.distance(t4) - d14
    
    t4.fd(dx4)
    

def tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot):
    '''Dado uma trajetória t, traça as demais trajetórias dos veículos t2,t3,t4,t5,t6'''
    
    tract0(t,t4)

    xc, yc = t.pos()
    x_t4,y_t4 = t4.pos()
    
    for k in [t2,t3,t5,t6]:
        k.setheading(t4.heading())

    a_rad = radians(t4.heading())
    
    x_t2 = xc - (lfrontal/2)*sin(a_rad)+ eixof*cos(a_rad)
    y_t2 = yc + (lfrontal/2)*cos(a_rad)+ eixof*sin(a_rad)
    t2.goto(x_t2,y_t2)

    x_t3 = xc + (lfrontal/2)*sin(a_rad)+ eixof*cos(a_rad)
    y_t3 = yc - (lfrontal/2)*cos(a_rad)+ eixof*sin(a_rad)
    t3.goto(x_t3,y_t3)        

    x_t5 = x_t4 + (ltraseira/2)*sin(a_rad)- eixot*cos(a_rad)
    y_t5 = y_t4 - (ltraseira/2)*cos(a_rad)- eixot*sin(a_rad)
    t5.goto(x_t5,y_t5)    

    x_t6 = x_t4 - (ltraseira/2)*sin(a_rad)- eixot*cos(a_rad)
    y_t6 = y_t4 + (ltraseira/2)*cos(a_rad)- eixot*sin(a_rad)
    t6.goto(x_t6,y_t6)
    

while t.heading() <= curva:
    if alfa2 < alfa :
        alfa2 = alfa2+0.1
    else:
        alfa2 = alfa
 
    t.lt(alfa2)
    t.fd(dx)
    tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot)

delta_head = t.heading()- curva

while delta_head > 0.1:
    delta_head = t.heading()- curva
    t.setheading(curva + 0.9*delta_head)
    t.fd(dx)
    tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot)
 

for _ in range(10):
    t.setheading(curva)
    t.fd(dx)
    tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot)
