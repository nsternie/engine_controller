import sys

packet_id = int(sys.argv[1])
target_id = int(sys.argv[2])
command_id = int(sys.argv[3])
num_args = int(sys.argv[4])



MASK = 0xff

out = [0] * (8+num_args*4+2)
out[0] = packet_id >> 8 & MASK
out[1] = packet_id & MASK
out[2] = target_id >> 8 & MASK
out[3] = target_id & MASK
out[4] = command_id >> 8 & MASK
out[5] = command_id & MASK
out[6] = num_args >> 8 & MASK
out[7] = num_args & MASK

args = []
for n in range(num_args):
	temp = int(sys.argv[5+n])
	out[8+4*n] = temp >> 24 & MASK
	out[9+4*n] = temp >> 16 & MASK
	out[10+4*n] = temp >> 8 & MASK
	out[11+4*n] = temp & MASK

checksum_0 = 0
checksum_1 = 0


f = open("command.bin", "wb+")
f.write(bytes(out))
f.close()