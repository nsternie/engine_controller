data = [ 0,
1,
0,
1,
0,
0,
0,
4,
0,
0,
0,
0,
0,
0,
0,
1,
0,
0,
0,
2,
0,
0,
0,
3]

a = 0
b = 0
for n in range(int(len(data)/2)):
	a ^= data[2*n]
	b ^= data[2*n+1]

print(a)
print(b)