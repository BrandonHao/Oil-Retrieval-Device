print("Input stationary gyro data:")

values = []
while True:
    value = input()
    if(value == ""):
        break
    values.append(float(value))

print("Offset: ")
print(sum(values)/len(values))
