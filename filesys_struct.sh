#!/bin/bash

VAR1="lab0"
README="README_"
AUX="/"
MD=".md"

for i in `seq 0 2`;
do
	LAB=$VAR1$i
	mkdir $LAB
	echo "Created directory: " $LAB
	touch $LAB$AUX$README$LAB$MD
        echo "Created file: " $LAB$AUX$README$LAB$MD	
done
