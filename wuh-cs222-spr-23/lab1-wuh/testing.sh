#!/bin/bash
LABN=lab1
CORRECT_TESTS=0
NUM_TESTS=35
declare -a inputfilelist=("add.x" "addi.x" "adds.x" "addis.x" "cbnz.x" "cbz.x" "and.x" "ands.x" 
	"eor.x" "orr.x" "ldur.x" "ldurb.x" "ldurh.x" "lsli.x" "lsri.x" "movz.x" "stur.x" "sturb.x" 
	"sturh.x" "sub.x" "subi.x" "subs.x" "subsi.x" "mul.x" "cmp.x" "br.x" "b.x" "beq.x" "bne.x" 
	"bgt.x" "blt.x" "bge.x" "ble.x" "countdigits.x" "fibonacci.x" "accumulation_20.x" "accumulation_210.x")
echo "lab1-tests.txt"  >> "lab1-tests.txt"
echo "---------"  >> "lab1-tests.txt"
echo "Section 1: individual instructions tests"  >> "lab1-tests.txt"
for inputfile in "${inputfilelist[@]}";
do
	echo "$inputfile"
	value=1
	i=1
	mismatch=0
	cycle=1
	
	while [ $i -eq $value ]
		do
			timeout 5 echo "input 11 2\n input 12 6\n input 20 400008\n input 1 128\n input 10 10000000\n go\n rdump\n mdump 0x10000000 0x100000ff" | timeout 1 ./sim inputs/${inputfile} > reference.txt
			cd src
			timeout 5 echo "input 11 2\n input 12 6\n input 20 400008\n input 1 128\n input 10 10000000\n go\n rdump\n mdump 0x10000000 0x100000ff" | timeout 1 ./sim ../inputs/${inputfile} > actual.txt
			cd .. # we are comparing the dumpsim files
			echo "------------------cycle $cycle---------------" >> diff_${inputfile}.txt
			value2=$(echo "$value2" | grep -r 'halted' reference.txt)
			value3=$(echo "$value3" | grep -r 'halted' src/actual.txt)
			if [[ -z "${value2// }" ]] && [[ -z "${value3// }" ]] # if length of string is 0
				then
					cycle=$((cycle+1))
				else		
					i=2			
			fi
			diff ./dumpsim ./src/dumpsim >> diff_final_${inputfile}.txt
		done
		if [ -s diff_final_${inputfile}.txt ] # if the file is not empty
		then
			if [ ! -f "lab1-tests.txt" ]; then
				touch "lab1-tests.txt"
		   fi
		   echo "Incorrect test: ${inputfile}" >> "lab1-tests.txt"
		else
			let "CORRECT_TESTS++" # diff file is empty, indicating that the test was passed
		fi
done
echo "Correct tests: $CORRECT_TESTS"  >> "lab1-tests.txt"
NUM_INCORRECT=$((NUM_TESTS - CORRECT_TESTS))
echo "Incorrect tests: $NUM_INCORRECT"  >> "lab1-tests.txt"
