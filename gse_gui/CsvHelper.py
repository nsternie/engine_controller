import csv



class CsvHelper:

    def loadCsv(self, filename):
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            csv_list = list(reader)
        csvDataArr = [len(csv_list), len(csv_list[1]), csv_list]

        # Returns data within csv file as follows:
        # csvDataArr[0] -> Number of rows in csv
        # csvDataArr[1] -> Number of cols in csv
        # csvDataArr[2][0] -> First row
        # csvDataArr[2][1] -> Second row
        # csvDataArr[2][N] -> N row
        return csvDataArr
