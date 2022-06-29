#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import re
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0) 
tempao_inicial = time.time()
marcela_altura = list()
marcela_tempo = list()
tempao = time.time()
while ( tempao - tempao_inicial < 150):
   data=ser.readline()
   print("where is perry?")
   print(data)
   f = open('myfile.txt', 'a')
   data=str(data)
   data = re.sub('[^0-9.]', '', data)
   try: 
    datafloat = float(data)
   
    marcela_altura.append(float(datafloat))
   except:
    marcela_altura.append(0.0)

   skipp ='\n'
   auau = 'auau'
   f.write(data)
   f.write(" ")
   tempao = time.time()
   f.write(str(tempao - tempao_inicial))
   marcela_tempo.append(tempao - tempao_inicial)
   f.write(skipp)
   f.close()
   time.sleep(0.5)
print(marcela_altura)
print(marcela_tempo)

plt.plot(marcela_tempo, marcela_altura)
plt.xlabel('Tempo')
plt.ylabel('PI')
plt.legend()
plt.show()
  