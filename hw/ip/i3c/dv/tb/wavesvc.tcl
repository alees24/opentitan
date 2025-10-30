global vpd_fid
set vpd_fid [dump -file "waves.vpd" -type VPD]
dump -add "tb" -fid $vpd_fid -depth 0
run
