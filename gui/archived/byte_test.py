import struct
import codecs

b = b'\x02\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c'

# print(type(b))
print("Stuffed: ",b)

#l = b.split(b'')
#print(l)
#l[2] = b'\x12'
		
unstuffed = b''
index = int(b[0])
for n in range(1, len(b)):
	temp = b[n:n+1]
	if(n == index):
		index = int(b[n])+n
		temp = b'\n'
	unstuffed = unstuffed + temp	

b = unstuffed

print("Unstuffed: ",Unstuffed	)