#!/bin/bash

echo "Updating the model ..."

cp ../../Models/model/tangram_cae.pth .
cp ../../Models/model/shapes.pt .

sleep 1s
echo "Model updated"
