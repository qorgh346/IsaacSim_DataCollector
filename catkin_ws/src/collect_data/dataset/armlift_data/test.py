import os
import json

# Input folder containing JSON files
input_folder = ".\\palletizer_data"

# Output file
output_file = "merged_data.json"


def merge_json_files(input_folder, output_file):
    merged_data = {}
    x = os.listdir(input_folder)
    # Get a sorted list of all JSON files in the folder
    json_files = sorted(
        [f for f in os.listdir(input_folder) if f.startswith("data_") and f.endswith(".json")],
        key=lambda x: int(x.split("_")[1].split(".")[0])  # Extract numeric part for sorting
    )

    for file in json_files:
        file_path = os.path.join(input_folder, file)

        # Extract timestamp from filename
        timestamp = file.split("_")[1].split(".")[0]

        # Read JSON file
        with open(file_path, "r") as f:
            data = json.load(f)

        # Add data to the merged dictionary
        merged_data[f"timestamp_{timestamp}"] = data

    # Write merged data to output file
    with open(output_file, "w") as f:
        json.dump(merged_data, f, indent=4)

    print(f"Merged {len(json_files)} files into {output_file}")


# Run the function
merge_json_files(input_folder, output_file)
