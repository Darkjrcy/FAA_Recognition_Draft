#!/bin/bash
echo "==== Downloading models.zip from Google Drive ===="

# Dowload and unzip the MOdels folder
# Google Drive file ID: 1612k0ajAOZc07ogzQaVJj58sSxcUIGob
wget --no-check-certificate "https://drive.google.com/uc?export=download&id=1612k0ajAOZc07ogzQaVJj58sSxcUIGob" -O models.zip

echo "=== Unzipping the files into the models folder ==="
unzip -o models.zip -d src/plane_description/models/


# Dowload and unzip the data YOLO folder
# Google Drive file ID: 1chHuow8cOx5b5a7lb4I7q1iILFkzfIup
wget --no-check-certificate "https://drive.google.com/uc?export=download&id=1chHuow8cOx5b5a7lb4I7q1iILFkzfIup" -O data.zip

echo "=== Unzipping the files into the data folder ==="
unzip -o models.zip -d src/airplane_recognition/

# Delete the zip files:
echo "=== clean both zip files==="
rm models.zip
rm data.zip
