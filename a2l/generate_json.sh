#./get_info.py ./VBU0401_02_a_Rep.a2l -o ./vbu.json -p TQD_trqTrqSetECO_MAP_v CrCtl_facDragSpeed_MAP_v TQD_flgBrkRgnConfig_C
#curl --header "PRIVATE-TOKEN: 5uRNNZX5gzTv2ta9xNoz" "https://gitlab.newrizon.work/api/v4/projects/52/repository/files/Mule%20Car%2FV0422%2FVBU0421_1_Rep.a2l/raw?ref=master" > VBU.a2l

# ./get_info.py VBU.a2l -o ./vbu.json -i ./init_value.json -p TQD_trqTrqSetECO_MAP_v
#VBU.a2l \
./get_info.py \
	VBU_L045A_CWP_05_09T_AImode_220407_1904.a2l \
	-o ./vbu.json \
	-i ./init_value.json \
	-p TQD_trqTrqSetNormal_MAP_v
# ./get_info.py VBU.a2l -o ./vbu.json -i ./init_value.json -p TQD_trqTrqSetNormal_MAP_v TQDIS_trqIncGradECORA_MAP_v
#./get_info.py VBU.a2l -o ./vbu.json -i ./init_value.json -p TQDIS_trqIncGradECORA_MAP_v
cp ./vbu.json ../xcp_driver/json/example.json
cp ./vbu.json ../xcp_driver/json/upload.json
cp ./vbu.json ../xcp_driver/json/download.json

#./xcp_send.py vbu.json -o out.json
