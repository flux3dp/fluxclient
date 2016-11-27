sudo python3 setup.py develop
echo "G2F"
flux_g2f -i ~/Downloads/octopus.gcode ~/Downloads/octopus.fc > ~/Downloads/log.txt
echo "F2G"
flux_f2g -i ~/Downloads/octopus.fc ~/Downloads/octopus_output.gcode
