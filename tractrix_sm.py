import turtle
from math import *
from tractrix import tract1, tract0,pino_rei


#curva = 180
t = turtle.Turtle()
wn = turtle.Screen()
#wn.setworldcoordinates(-100,-100, 100,100)
wn.tracer(100)

def trajetoria_sm(curva, dx):


    # Dados do Cavalo
    lfrontal = 2.49 #   Largura frontal
    eixof = 1.45 # Recuo do eixo em relação a frente do veículo
    d_eixo = 4.2 # Distância entre eixos
    ltraseira = 2.49 #   Largura traseira
    eixot = 0.71 # Recuo do eixo em relação a traseira do veículo

    # Dados do Reboque

    lrbq = 2.6 #Largura do reboque
    d4pr = 0.35 #Distância do eixo traseiro do cavalo mecânico ao pino em m (O pino fica a frente do eixo traseiro do cavalo mecânico)
    efr = 2 #Recuo do 1ºeixo (frontal) do reboque
    etr = 4.75 #Recuo do 2ºeixo (traseiro) do reboque
    dexr = 8.1 #Distância entre os eixos do reboque

    # estsm = 15 # Angulo máximo de Esterçamento semi-reboque/CM (Graus)

    alfa = 25 #25 #ângulo máximo de esterçamento : alfa (Graus)
    #dx = 0.5 # Discretização, indicando o acrescimo de posição a cada iteração
    raio_inic = 22 # Raio que se deseja adotar, só aceito se for maior que o raio mínimo
    rmin = d_eixo / sin(radians(alfa)) # Raio mínimo da composição
    if raio_inic > rmin:
        rmin = raio_inic
    
    teta = degrees(2*asin(dx/(2*rmin)))# Angulo de giro pata a trajetória circular de menor raio definida


    # Eixos e extremidades do cavalo
    t2 = t.clone() # Roda dianteira direita
    t2.up()
    t2.goto(eixof,-lfrontal/2)


    t3 = t.clone() # Roda dianteira esquerda
    t3.up()
    t3.goto(eixof,+lfrontal/2)


    t4 = t.clone() # Eixo traseiro
    t4.up()
    t4.goto(-d_eixo,0)


    t5 = t.clone() # Lateral traseira direita
    t5.up()
    t5.goto(t4.xcor()-eixot,-ltraseira/2)


    t6 = t.clone() # Lateral traseira esquerda
    t6.up()
    t6.goto(t4.xcor()-eixot,ltraseira/2)


    ## Eixos e extremidades do reboque ##

    t7 = t.clone() # Eixo frontal do reboque
    t7.up()
    t7.goto(t4.xcor()+d4pr,0)


    t8 = t.clone() # Extremidade direita forntal do reboque
    t8.up()
    t8.goto(t7.xcor()+efr,-lrbq/2)

    t9 = t.clone() # Extremidade equerda forntal do reboque
    t9.up()
    t9.goto(t8.xcor(),lrbq/2)

    t10 = t.clone() # Eixo traseiro do reboque
    t10.up()
    t10.goto(t7.xcor()-dexr,0)
    d710 = dexr

    t11 = t.clone() # Lateral traseira direita
    t11.up()
    t11.goto(t10.xcor()-etr,-lrbq/2)

    t12 = t.clone() # Lateral traseira esquerda
    t12.up()
    t12.goto(t11.xcor(),+lrbq/2)


    d14 = d_eixo # Distância entre t1(t) e t4
    d47 = t4.distance(t7)
    d4_10 = t10.distance(t4) # Distância entre t4 e t10
    d17 = t.distance(t7)


    #Menor distância permitida entre os pontos 1 e 10 com esterçamento máximo do semireboque
    md110 = (d17**2 + dexr**2)**0.5 # + 2*d17*dexr*cos(radians(estsm)))**(1/2)

    colors = ["black","green","green","black","red","red"]*2

    for k,j in zip([t,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12],colors):
        k.color(j)
        k.speed("fastest")
        k.down()
        k.begin_poly()

    t7.up()
    t7.ht()

    alfa2 = 0.1*teta
    while t.heading() < (curva - teta):
        if t.distance(t10) > md110: #Verificando o exterçamento máximo do Semireboque
            if abs(alfa2) < abs(teta) :
                t.lt(alfa2)
                alfa2 = alfa2 + 0.1*teta

            else:
                t.lt(teta)
##                if t.heading() > curva - teta:
##                    t.setheading(curva - teta)
##                    print('passou')
            t.fd(dx)
        else:
            print("Semi reboque estercado, aumentar discretização 'dx'")
            print(t.distance(t10))
            t.fd(dx) # Se o semireboque estiver esterça, o veículo vai alinhar e depois girar
        
        tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14)

        pino_rei(t,t4,t7,d4pr) # t7 - Pino/eixo do semireboque fica a frente do eixo traseiro do cavalo mecânico    
        tract1(t7,t8,t9,t10,t11,t12,lrbq,efr,lrbq,etr,d710)
        t7.setheading(t8.heading())

    delta_head = curva - t.heading()
    
    for cnt in range(10):
        t.lt(delta_head/10)
        t.fd(dx)
        tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14)
        pino_rei(t,t4,t7,d4pr) # t7 - Pino/eixo do semireboque fica a frente do eixo traseiro do cavalo mecânico
        tract1(t7,t8,t9,t10,t11,t12,lrbq,efr,lrbq,etr,d710)
        t7.setheading(t8.heading())
    
        ##    while t10.heading() < curva - 0.1:
        ##        t.fd(dx)
        ##        tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14)
        ##        pino_rei(t,t4,t7,d4pr) # t7 - Pino/eixo do semireboque fica a frente do eixo traseiro do cavalo mecânico
        ##        tract1(t7,t8,t9,t10,t11,t12,lrbq,efr,lrbq,etr,d710)
        ##        t7.setheading(t8.heading())

    
    lista_coord = []
    for k,j in zip([t,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12],colors):
        k.end_poly()
        lista_coord.append([[i,j] for i,j in k.get_poly()])
    
    c = 0
    out = open('trajetorias.scr', 'w')
    out.write('OSNAP OFF\n')
    for k in lista_coord:
        c+=1
        out.write('_COLOR '+str(c)+'\n')
        out.write('_PLINE\n')
        for coord in k:
            out.write(f'{coord[0]:.3f},{coord[1]:.3f}\n')
        out.write('\n')
    out.close()


########################### depois arrumar 

# Dados do Cavalo
lfrontal = 2.49 #   Largura frontal
eixof = 1.45 # Recuo do eixo em relação a frente do veículo
d_eixo = 4.2 # Distância entre eixos
ltraseira = 2.49 #   Largura traseira
eixot = 0.71 # Recuo do eixo em relação a traseira do veículo

# Dados do Reboque

lrbq = 2.6 #Largura do reboque
d4pr = 0.35 #Distância do eixo traseiro do cavalo mecânico ao pino em m (O pino fica a frente do eixo traseiro do cavalo mecânico)
efr = 2 #Recuo do 1ºeixo (frontal) do reboque
etr = 4.75 #Recuo do 2ºeixo (traseiro) do reboque
dexr = 8.1 #Distância entre os eixos do reboque

# estsm = 15 # Angulo máximo de Esterçamento semi-reboque/CM (Graus)

alfa = 25 #25 #ângulo máximo de esterçamento : alfa (Graus)
dx = 0.5 # Discretização, indicando o acrescimo de posição a cada iteração
raio_inic = 22 # Raio que se deseja adotar, só aceito se for maior que o raio mínimo
rmin = d_eixo / sin(radians(alfa)) # Raio mínimo da composição
if raio_inic > rmin:
    rmin = raio_inic

teta = degrees(2*asin(dx/(2*rmin)))# Angulo de giro pata a trajetória circular de menor raio definida


# Eixos e extremidades do cavalo
t2 = t.clone() # Roda dianteira direita
t2.up()
t2.goto(eixof,-lfrontal/2)


t3 = t.clone() # Roda dianteira esquerda
t3.up()
t3.goto(eixof,+lfrontal/2)


t4 = t.clone() # Eixo traseiro
t4.up()
t4.goto(-d_eixo,0)


t5 = t.clone() # Lateral traseira direita
t5.up()
t5.goto(t4.xcor()-eixot,-ltraseira/2)


t6 = t.clone() # Lateral traseira esquerda
t6.up()
t6.goto(t4.xcor()-eixot,ltraseira/2)


## Eixos e extremidades do reboque ##

t7 = t.clone() # Eixo frontal do reboque
t7.up()
t7.goto(t4.xcor()+d4pr,0)


t8 = t.clone() # Extremidade direita forntal do reboque
t8.up()
t8.goto(t7.xcor()+efr,-lrbq/2)

t9 = t.clone() # Extremidade equerda forntal do reboque
t9.up()
t9.goto(t8.xcor(),lrbq/2)

t10 = t.clone() # Eixo traseiro do reboque
t10.up()
t10.goto(t7.xcor()-dexr,0)
d710 = dexr

t11 = t.clone() # Lateral traseira direita
t11.up()
t11.goto(t10.xcor()-etr,-lrbq/2)

t12 = t.clone() # Lateral traseira esquerda
t12.up()
t12.goto(t11.xcor(),+lrbq/2)


d14 = d_eixo # Distância entre t1(t) e t4
d47 = t4.distance(t7)
d4_10 = t10.distance(t4) # Distância entre t4 e t10
d17 = t.distance(t7)


#Menor distância permitida entre os pontos 1 e 10 com esterçamento máximo do semireboque
md110 = (d17**2 + dexr**2)**0.5 # + 2*d17*dexr*cos(radians(estsm)))**(1/2)

colors = ["black","green","green","black","red","red"]*2

for k,j in zip([t,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12],colors):
    k.color(j)
    k.speed("fastest")
    k.down()
    k.begin_poly()

t7.up()
t7.ht()

ref_arquivo = open("coord_projmod.txt","r",encoding='utf-8-sig')
lista_de_coord = ref_arquivo.readlines()
ref_arquivo.close()

lcoor = []
for k in lista_de_coord:
    k = k.replace('\n','')
    if len(k) > 0:
        w = k.split(',')
        w[0] = eval(w[0])
        w[1] = eval(w[1])

    lcoor.append(w)

k = 0
while k < len(lcoor):
    if t.distance(lcoor[k][0],lcoor[k][1]) < 10*dx:
        t.goto(lcoor[k][0],lcoor[k][1])
        k+=1
    else:
        t.setheading(t.towards(lcoor[k][0],lcoor[k][1]))
        t.fd(dx)
    t.setheading(t4.towards(t))
    tract1(t,t2,t3,t4,t5,t6,lfrontal,eixof,ltraseira,eixot,d14)
    pino_rei(t,t4,t7,d4pr) # t7 - Pino/eixo do semireboque fica a frente do eixo traseiro do cavalo mecânico
    tract1(t7,t8,t9,t10,t11,t12,lrbq,efr,lrbq,etr,d710)
    t7.setheading(t8.heading())

lista_coord = []
for k,j in zip([t,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12],colors):
    k.end_poly()
    lista_coord.append([[i,j] for i,j in k.get_poly()])

c = 0
out = open('trajetorias.scr', 'w')
out.write('OSNAP OFF\n')
for k in lista_coord:
    c+=1
    out.write('_COLOR '+str(c)+'\n')
    out.write('_PLINE\n')
    for coord in k:
        out.write(f'{coord[0]:.3f},{coord[1]:.3f}\n')
    out.write('\n')
out.close()

wn.exitonclick()
