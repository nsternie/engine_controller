import csv



class CsvHelper:

    def loadCsv(self, filename):
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            csv_list = list(reader)
        csvDataArr = [len(csv_list), csv_list]

        # Returns data within csv file as follows:
        # csvDataArr[0] -> Number of cols in csv
        # csvDataArr[1][0] -> First col
        # csvDataArr[1][1] -> Second Col
        # csvDataArr[1][N] -> N Col
        return csvDataArr
