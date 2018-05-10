# metricEvaluations

General pipeline for working with this repository:

1. Add rosbag files from the last run into a folder outside the repository (or add them to your git .ignore)
2. Use ```bag_to_csv.py``` to convert the rosbag file to a .csv using the command: ```python bag_to_csv.py /path/to/file.bag```
3. Use ```plots.py``` to generate plots for entropy and map coverage

Note: this repository uses pandas for parsing and processing the .csv file (https://pandas.pydata.org/)
