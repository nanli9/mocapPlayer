import sys

jointName = sys.argv[1]
amc_file_1 = sys.argv[2]
amc_file_2 = sys.argv[3]
amc_file_3 = sys.argv[4]
outputFile = sys.argv[5]
count = 0

#input
input = open(amc_file_1,"r")
data = input.readlines()
array1 = []
for line in data:
    if jointName in line:
        array1.append(line.split(" ")[1])
input.close()

file1 = open(amc_file_2,"r")
data = file1.readlines()
array2 = []
for line in data:
    if jointName in line:
        array2.append(line.split(" ")[1])
file1.close()

file2 = open(amc_file_3,"r")
data = file2.readlines()
array3 = []
for line in data:
    if jointName in line:
        array3.append(line.split(" ")[1])
file2.close()

f = open(outputFile, "w")
x = "x = ["
y = "y = ["
for angles in array1:
    count = count + 1
    if count >=600 and count <=800:
        x = x+str(count)+" "
        y = y+angles+" "
x += "];\n"
y += "];\n"

count = 0
y1 = "y1 = ["
for angles in array2:
    count = count + 1
    if count >=600 and count <=800:
        y1 = y1+angles+" "
y1 += "];\n"

count = 0
y2 = "y2 = ["
for angles in array3:
    count = count + 1
    if count >=600 and count <=800:
        y2 = y2+angles+" "
y2 += "];\n"

f.write(x)
f.write(y)
f.write(y1)
f.write(y2)
f.write("plot(x,y,'r',x,y1,'g',x,y2,'b')\n")
f.write("xlabel('frames')\n")
f.write("ylabel('X-Axis-Angles')\n")

f.write("legend('input','" + amc_file_2 + "','"+amc_file_3+"')\n")
f.close()