#!/bin/bash
source $HOME/.bashrc

if [ -d "$HBP" ] 
then
    echo "Directory $HBP exists." 

	echo
	echo =============================
	echo Deploying Floreano Experiment
	echo =============================

	cp -H -v -u Models/brain_model/floreano.py $HBP/Models/brain_model

	cp -H -v -r -u Models/floreano_box $HBP/Models

	cp -H -v -r -u Experiments/floreano $HBP/Experiments

	sed -i -e '/floreano/ d' $HBP/Experiments/_rpmbuild/folders.txt
	echo "floreano" >> $HBP/Experiments/_rpmbuild/folders.txt

	cd $HBP/Models
	./create-symlinks.sh
	cd $HBP/gzweb
	./deploy-gzbridge-nrp.sh
	cd $HBP/user-scripts
	./configure_nrp
else
    echo "Error: Directory $HBP does not exists."
fi
