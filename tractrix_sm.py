import turtle
from math import *


curva = 150

# Dados do Cavalo
lfrontal = 2.49 #   Largura frontal
eixof = 0.91 # Recuo do eixo em relação a frente do veículo
d_eixo = 4 # Distância entre eixos
ltraseira = 2.49 #   Largura traseira
eixot = 0.91 # Recuo do eixo em relação a frente do veículo

# Dados do Reboque

lrbq = 2.49 #Largura do reboque
d4pr = 0.2 #Distância do eixo traseiro ao pino em m
dp4er = 0.8 # Distância do pino rei ao primeiro eixo do reboque
efr = 0.9 #Recuo do 1ºeixo do reboque
etr = 0.9 #Recuo do 1ºeixo do reboque
dexr = 6 #Distância entre os eixos do reboque

estsm = 15 # Angulo máximo de Esterçamento semi-reboque/CM (Graus)

alfa = 25 #ângulo máximo de esterçamento : alfa (Graus)
dx = 1 #Discretização, indicando o acrescimo de posição a cada iteração


t = turtle.Turtle()

wn = turtle.Screen()

wn.setworldcoordinates(-20,-5, 45,60)


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


t5 = t.clone() # Lateral traseira direita
t5.up()
t5.goto(-d_eixo-eixot,-ltraseira/2)


t6 = t.clone() # Lateral traseira esquerda
t6.up()
t6.goto(-d_eixo-eixot,ltraseira/2)


## Eixos e extremidades do reboque ##

t7 = t.clone() # Eixo frontal do reboque
t7.up()
t7.goto(t4.xcor()-(d4pr+dp4er),0)


t8 = t.clone() # Extremidade esquerda forntal do reboque
t8.up()
t8.goto(t7.xcor()+efr,+lrbq/2)

t9 = t.clone() # Extremidade direita forntal do reboque
t9.up()
t9.goto(t8.xcor(),-lrbq/2)

t10 = t.clone() # Eixo traseiro do reboque
t10.up()
t10.goto(t7.xcor()-dexr,0)
d710 = dexr

t11 = t.clone() # Lateral traseira esquerda
t11.up()
t11.goto(t10.xcor()-etr,-lrbq/2)

t12 = t.clone() # Lateral traseira direita
t12.up()
t12.goto(t11.xcor(),+lrbq/2)


d14 = -t4.xcor() # Distância entre t1(t) e t4
d47 = d4pr+dp4er
d4_10 =  t4.xcor() - t10.xcor() # Distância entre t4 e t10


#Menor distância pemritida entre os pontos 1 e 10 com esterçamento máximo do semireboque
md110 = (d14**2 + d4_10**2 + 2*d14*d4_10*cos(radians(estsm)))**(1/2)


colors = ["black","green","green","black","red","red"]*2
c = 0


for k in [t,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12]:
    k.color(colors[c])
    c = c + 1
    k.speed("fastest")
    k.down()

t7.up()
t7.ht()

for _ in range(10):
    for k in [t,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12]:
        k.fd(dx)



alfa2 = 0
k = 0
a = 92
b = a

def tract0(t,t4,d):
    
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


    dx4 = t.distance(t4) - d
    t4.fd(dx4)
    

def tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14):
    '''Dado uma trajetória t, traça as demais trajetórias dos veículos t2,t3,t4,t5,t6'''
    
    tract0(t,t4,d14)

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

    if t.distance(t10) >= md110-dx: #Verificando o exterçamento máximo do Semireboque
        if alfa2 < alfa :
            alfa2 = alfa2+0.1
        else:
            alfa2 = alfa
        t.lt(alfa2)
    else:
        print("estercado")
        print(t.distance(t10))
        
    t.fd(dx)
    tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14)

    tract0(t4,t7,d47)    
    tract1(t7,t8,t9,t10,t11,t12,lrbq,efr,lrbq,etr,d710)


delta_head = t.heading()- curva

while delta_head > 0.1:
    delta_head = t.heading()- curva
    t.setheading(curva + 0.9*delta_head)
    t.fd(dx)
    tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14)
    tract0(t4,t7,d47)    
    tract1(t7,t8,t9,t10,t11,t12,lrbq,efr,lrbq,etr,d710)    

for _ in range(10):
    t.setheading(curva)
    t.fd(dx)
    tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14)
    tract0(t4,t7,d47)    
    tract1(t7,t8,t9,t10,t11,t12,lrbq,efr,lrbq,etr,d710) 
