#!/bin/bash
source $HOME/.bashrc

if [ -d "$HBP" ] 
then
    echo "Directory $HBP exists." 

	echo
	echo =============================
	echo Grabing Floreano Experiment
	echo =============================

	cp -H -v -u $HBP/Models/brain_model/floreano.py Models/brain_model

	cp -H -v -r -u $HBP/Models/floreano_box Models

	cp -H -v -r -u $HBP/Experiments/floreano Experiments

else
    echo "Error: Directory $HBP does not exists."
fi
