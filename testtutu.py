import turtle

t = turtle.Turtle()
t2 = turtle.Turtle()

t.lt(30)
t.fd(100)


t2.lt(78)
t2.fd(100)


print(t.towards(t2))
print(t2.towards(t))
