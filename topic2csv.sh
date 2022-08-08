# save into .csv files
rostopic echo -p /imu/data > data.csv
rostopic echo -p /imu/data_raw > data_raw.csv
