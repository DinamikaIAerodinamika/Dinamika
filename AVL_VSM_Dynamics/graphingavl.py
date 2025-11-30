import matplotlib.pyplot as plt
with open ("Rear_Run59_an_kc_rear_axle1_rear.sdf", "r") as file:
    counter=1
    listofvalues=[]
    for i in file:
        if counter>9 and (counter-9)%94==4:
            c2=1
            leftvalue=""
            rightvalue=""
            meanvalue=""
            check1=False
            check2=False
            check3=False
            for j in i:
                if c2>44 and (j.isnumeric() or j=="." or j=="-"):
                    check1=True
                    if check2==False:
                        leftvalue+=j
                    elif check3==False:
                        rightvalue+=j
                    else:
                        meanvalue+=j
                elif check1==True and check2==False:
                    check2=True
                    check1=False
                elif check1==True and check2==True:
                    check3=True
                c2+=1
            listofvalues.append([leftvalue,rightvalue,meanvalue])
        counter+=1
    listofleftvalues=[float(i[0]) for i in listofvalues]
    listofrightvalues=[float(i[1]) for i in listofvalues]
    listofmeanvalues=[float(i[2]) for i in listofvalues]
    time=[i*0.1 for i in range(len(listofleftvalues))]
    plt.plot(time,listofleftvalues)
    plt.show()
    
    
