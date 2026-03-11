import ctypes

# Load the DLL
my_dll = ctypes.WinDLL("C:\\Users\\Casey\\Desktop\\LinUDP_V2_1_1_0_20210617\\Examples\\LabView\\LinUDP_V2\\LinUDP.dll")  # Replace "my_dll.dll" with the actual name of your DLL

# Access a function from the DLL
#my_function = my_dll.LinUDP.ACL  # Replace "my_function" with the actual function name

# Call the function
#result = my_function()

#print(result)
functions = [name for name, _ in my_dll.__dict__.items()]

# Print the functions
for function in functions:
    print(function)