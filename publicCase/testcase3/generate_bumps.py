import random

def generate_bumps(chip_width, chip_height, num_bumps):
    bumps = set()
    while len(bumps) < num_bumps:
        x = random.randint(0, chip_width // 10 - 1) * 10
        y = random.randint(0, chip_height // 10 - 1) * 10
        bumps.add((x, y))

    # Convert to list with bump IDs
    return [(bump_id + 1, x, y) for bump_id, (x, y) in enumerate(bumps)]

def main():
    # Chip 1 dimensions
    chip1_width = 340
    chip1_height = 570

    # Chip 2 dimensions
    chip2_width = 340
    chip2_height = 570

    # Generate 500 total bumps, split evenly between two chips
    bumps_per_chip = 500

    # Generate bumps for Chip 1
    chip1_bumps = generate_bumps(chip1_width, chip1_height, bumps_per_chip)

    # Generate bumps for Chip 2
    chip2_bumps = generate_bumps(chip2_width, chip2_height, bumps_per_chip)

    # Write to file
    with open("bumps_output.txt", "w") as file:
        file.write(".c\n")
        file.write(f"3900 1100 {chip1_width} {chip1_height}\n")
        file.write(".b\n")
        for bump_id, x, y in chip1_bumps:
            file.write(f"{bump_id} {x} {y}\n")

        file.write(".c\n")
        file.write(f"11800 8010 {chip2_width} {chip2_height}\n")
        file.write(".b\n")
        for bump_id, x, y in chip2_bumps:
            file.write(f"{bump_id} {x} {y}\n")

if __name__ == "__main__":
    random.seed(42)  # For reproducibility
    main()
