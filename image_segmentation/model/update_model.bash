#!/bin/bash

echo "Updating the model ..."

cp ../../model/model/tangram_cae.pth .
cp ../../model/model/shapes.pt .

sleep 1s
echo "Model updated"
