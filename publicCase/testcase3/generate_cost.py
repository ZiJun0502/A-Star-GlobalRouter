import random

# Parameters
height = 1000  # Height of the grid
width = 1500   # Width of the grid
alpha = 1.1
beta = 1.5
gamma = 1.1
delta = 0.7
v = 3.5

# Generate the grid costs (random for this example)
def generate_grid_data(height, width):
    grid_data = []
    for _ in range(height):
        row = [random.randint(0, 10) for _ in range(width)]
        grid_data.append(row)
    return grid_data

# Writing the test case to a file
def write_test_case(filename, height, width, alpha, beta, gamma, delta, v):
    with open(filename, 'w') as file:
        # Writing the parameter values
        file.write(f".alpha {alpha}\n")
        file.write(f".beta {beta}\n")
        file.write(f".gamma {gamma}\n")
        file.write(f".delta {delta}\n")
        file.write(f".v\n")
        file.write(f"{v}\n")
        
        # Writing layer 1 grid
        file.write(".l\n")
        grid_data_layer_1 = generate_grid_data(height, width)
        for row in grid_data_layer_1:
            file.write(" ".join(map(str, row)) + "\n")
        
        # Writing another layer (layer 2) with similar random data
        file.write(".l\n")
        grid_data_layer_2 = generate_grid_data(height, width)
        for row in grid_data_layer_2:
            file.write(" ".join(map(str, row)) + "\n")

# Generate test case file
write_test_case('generated_test_case.txt', height, width, alpha, beta, gamma, delta, v)

print("Test case file generated successfully.")
