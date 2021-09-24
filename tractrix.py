import turtle
from math import *

t = turtle.Turtle()

wn = turtle.Screen()
wn.setworldcoordinates(-75,-5, 40,300)

l = 50 #Comprimento da barra ligando t a t2

t2 = t.clone()
t2.up()
t2.goto(-l,0)
t2.down()


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

t.lt(90)
t.speed("fastest")
for _ in range(l*6):
    t.fd(1)
    tract0(t,t2,l)
