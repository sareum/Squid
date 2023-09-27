import numpy as np
import csv
import json

def plot_and_save_data(plottingData, xAxisLabel, yAxisLabel, label, savingData, savingData_names,
                        filename, saveDir, display_plot = True, saveData = True, saveFig = True, figsize = (6,8)):

    if saveData:
        # Save the main data
        save_data = np.transpose(np.vstack(savingData))
        np.savetxt(saveDir + '/' + filename + ".csv", save_data, delimiter = ",")

        # Save a csv file which indicates the column names
        with open(saveDir + '/' + filename + "_col_name.txt", 'w') as f:
            writer = csv.writer(f)
            writer.writerow(savingData_names)


def save_to_json(filename, data):
    # Serializing json
    json_object = json.dumps(data, indent=4)

    # Writing to sample.json
    with open(filename + "_config.json", "w") as outfile:
        outfile.write(json_object)
