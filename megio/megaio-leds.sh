#/bin/bash
# A stupid little script to demonstrate/test 
# LED Card for Mega-IO & Raspberry Pi from
# https://www.sequentmicrosystems.com/
# free to do anyhing you want to do with it
# dezldog 16JUN18 

if [ $# -eq 0 ]
  then
    echo "Usage: megaio-leds.sh <seconds/100 to light each led>"
    exit;
fi

echo "Press 'q' to exit gracefully"

while :; do

	for lednum in {1..32};
		do 
			megaio -lw $lednum on;
			sleep ($1/100);
			megaio -lw $lednum off;
			
	done

	read -t 0.01 -rN 1 && [[ $REPLY == 'q' ]] && break

done

exit;
