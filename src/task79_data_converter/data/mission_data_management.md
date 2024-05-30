# Data Management

In order to save space in the repo, the data files are ignored by git.

The recommended organisation for mission data files is:

```
* task79_data_converter/
    * data/
        * mission_data/
            * test_X
                * file_1.csv
                * file_2.csv
            * test_Y
                * file_1.csv
                * file_2.csv
        * formatted_mission_data/
            * test_X
                * file_1.csv
                * file_2.csv
```

For information, mission data files (extracted from vehicle missions) can be found in `task79_data_converter/data/mission_data.zip` and need to be extracted following the recommended organisation. 

Additionally, formatted mission data files (output by the data converter) can be found in `task79_data_converter/data/formatted_mission_data.zip` and need to be extracted following the recommended organisation. 

If the relevant data files are not accessible, please send an email to the package author (cf. `task79_data_converter/package.xml`).
