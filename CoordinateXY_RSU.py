3241.23,3842.09
7376.61,3901.17
3300.30,939.94
import sys
x= [] # 3241.23 , 7376.61
y= [] # 939.94, 3842.09

l= 3241.23
while l <= 7376.61:
 x.append(l)
 l = l + 1000 # 1000 is radius of the RSU
 
 
d=3842.09
while d>= 939.94:
 y.append(d)
 d= d-1000

sys.stdout = open('RSUsLocation_xy.xml','wt')
print ("{0}{1}xml version={2} encoding={3}{4}{5}" .format('<','?','"1.0"','"iso-8859-1"','?','>'))
print ("{0}RSUs{1}" .format('<','>'))

b= 0 # for RSU id
for i in x:
 for j in y:
  print ("{0}poly id={1}{2}{3} type={4} center={5}{6},{7}{8} {9}{10}" .format('<','"RSU', b,'"','"RSU"','"',i,j,'"','/','>'))
  b = b + 1
 
print ("{0}{1}RSUs{2}" .format('<','/','>'))
 
 
