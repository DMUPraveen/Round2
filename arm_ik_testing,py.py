from math import asin, acos, atan,pi


a1 =0.253

a2 = 0.155
a3 = 0.135
a4 = 0.081
a5 = 0.105


def arm_ik(x,y,z):
    x1 = (x**2+y**2)**0.5
    y1 = y + a4 +a5 -a1
    a = a2
    b = a3
    c = (x1**2+y1**2)**0.5

    theta = acos((a*a+c*c-b*b)/(2*a*c))

    alpha = asin(z/x1)
    beta = pi/2 - theta
    gamma = theta -pi
    delta = -(pi +beta +gamma)
    epsilon = pi/2 + alpha

    return (alpha,beta,gamma,delta,epsilon)

def main():
    while True:
        x =float(input("give x:"))
        y =float(input("give y:"))
        z =float(input("give z:"))
        try:
            values = arm_ik(x,y,z)
            #print(values)
            print("alpha: {}, beta: {}, gamma {}, delta {}, epsilon {}".format(values[0],values[1],values[2],values[3],values[4]))
        except:
            print("invalid input")


    

def main2():
    file = open("positions.txt","w+")
    x =0
    y = 0
    z = 0
    for x in range(0,100):
        for y in range(0,100):
            for z in range(0,100):
                try:
                    values = arm_ik(x*0.01,y*0.01,z*0.01)
                    file.write("{},{},{} : {} \n".format(x*0.01,y*0.01,z*0.01,((x*0.01)**2+(y*0.01)**2+(z*0.01)**2)**0.5))
                except:
                    pass
    file.close()

    file.close()
                    

if __name__ == "__main__":
    main2()
