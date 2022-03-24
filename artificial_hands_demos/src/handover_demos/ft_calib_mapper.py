import csv

with open('/home/penzo/Desktop/calib_data_20220323_154354.csv') as csv_file:
  csv_reader = csv.reader(csv_file, delimiter=',')
  for row in csv_reader:
    print(row)