import sys
import os

for n in range(1, len(sys.argv)):
    print(("De-interlacing " + sys.argv[n] + " ... "))

    TARGET_ADDRESS_GROUND     =    100
    TARGET_ADDRESS_FLIGHT     =    101

    source_file = open(sys.argv[n], 'r')

    ground_file = open(sys.argv[n].strip(".csv")+"_ground.csv", 'w')
    flight_file = open(sys.argv[n].strip(".csv")+"_flight.csv", 'w')

    files = [ground_file, flight_file]



    header = True
    id_col = 0
    for line in source_file:
        if(header):
            split = line.split(',')
            for index,name in enumerate(split):
                if(name == 'BOARD_ID ()'):
                    id_col = index
                    print(id_col)
            for file in files:
                file.write(line)
            header = False
        else:
            if(id_col == 0):
                break
                break
            split = line.split(',')
            board_id = int(split[id_col])
            if(board_id == TARGET_ADDRESS_GROUND):
                ground_file.write(line)
            elif(board_id == TARGET_ADDRESS_FLIGHT):
                flight_file.write(line)
            else:
                print("Invalid board id: "+str(board_id))

    ground_file.close()
    flight_file.close()
    source_file.close()

    print("done")