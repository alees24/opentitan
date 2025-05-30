# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

send_msg "Designcheck 1-1" INFO "Checking design"

# Ensure the design meets timing
set slack_ns [get_property SLACK [get_timing_paths -delay_type min_max]]
send_msg "Designcheck 1-2" INFO "Slack is ${slack_ns} ns."

if [expr {$slack_ns < 0}] {
  send_msg "Designcheck 1-3" ERROR "Timing failed. Slack is ${slack_ns} ns."
}

# Enable bitstream identification via USR_ACCESS register.
set_property BITSTREAM.CONFIG.USR_ACCESS TIMESTAMP [current_design]

# Generate an MMI file for the given BRAM cells.
#
# Args:
#   filename:            Path to the output file.
#   mem_info:            Dictionary of dictionaries with following properties for each (inner) value:
#       brams:               A list of BRAM cells.
#       mem_type_regex:      The BRAM type regex, dividing the mem type and site, e.g. {(RAMB\d+)_(\w+)}.
#       fake_word_width:     If non-zero, pretend that $brams covers
#                            `fake_word_width` bits. Influences the values of the
#                            MMI's <AddressSpace> and <DataWidth> tags.
#       addr_end_multiplier: A coefficient applied to the address space. Influences
#                            the values of the MMI's <AddressSpace> and
#                            <AddressRange> tags.
#       schema:              Either "Processor" or "MemoryArray"
#   designtask_count:    A number used for logging with `send_msg`.
proc generate_mmi {filename mem_infos designtask_count} {
    send_msg "${designtask_count}-1" INFO "Dumping MMI to ${filename}"

    set workroot [file dirname [info script]]
    set filepath "${workroot}/${filename}"
    set fileout [open $filepath "w"]
    set part [get_property PART [current_design]]
    puts $fileout "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    puts $fileout "<MemInfo Version=\"1\" Minor=\"1\">"

    dict for {id mem_info} $mem_infos {
        dict with mem_info {
            if {[llen $brams] == 0} {
                send_msg "${designtask_count}-1" INFO "Cannot make MMI for zero BRAMs"
                return
            }

            set fake_slice_width [expr $fake_word_width / [llen $brams]]

            # Calculate the overall address space.
            set space 0
            set width 0
            foreach inst [lsort -dictionary $brams] {
                set slice_begin [get_property ram_slice_begin [get_cells $inst]]
                set slice_end [get_property ram_slice_end [get_cells $inst]]
                if {$slice_begin eq {} || $slice_end eq {}} {
                    send_msg "${designtask_count}-2" ERROR "Extraction of ${filename} information failed."
                }
                set slice_width [expr {$slice_end - $slice_begin + 1}]
                if {$slice_width < $fake_slice_width} {
                    set slice_end [expr {$slice_begin + $fake_slice_width - 1}]
                    set slice_width $fake_slice_width
                }
                set addr_begin [get_property ram_addr_begin [get_cells $inst]]
                set addr_end [get_property ram_addr_end [get_cells $inst]]
                if {$addr_begin eq {} || $addr_end eq {}} {
                    send_msg "${designtask_count}-3" ERROR "Extraction of ${filename} MMI information failed."
                }

                # Calculate total number of bits.
                set space [expr {$space + ($addr_end - $addr_begin + 1) * $slice_width}]
                set width [expr {$width + $slice_width}]
                set last_slice_width $slice_width
            }
            set space [expr {($space * $addr_end_multiplier / 8) - 1}]

            # Generate the MMI.
            if { $schema eq "Processor" } {
                puts $fileout "  <Processor Endianness=\"Little\" InstPath=\"$id\">"
                puts $fileout "    <AddressSpace Name=\"dummy_addrspace\" Begin=\"0\" End=\"$space\">"
                puts $fileout "      <BusBlock>"
            } else {
                puts $fileout "  <MemoryArray InstPath=\"$id\" MemoryPrimitive=\"auto\" MemoryConfiguration=\"enabled_configuration\">"
                # Memory type could be retrieved from the RTL_RAM_TYPE property, if desired,
                # but we hard-code it here for now.
                puts $fileout "    <MemoryLayout Name=\"$id\" CoreMemory_Width=\"$width\" MemoryType=\"RAM_SP\">"
            }

            foreach inst [lsort -dictionary $brams] {
                set loc [get_property LOC [get_cells $inst]]
                set loc_matches [regexp $mem_type_regex $loc loc_match loc_prefix loc_suffix]
                if {$loc_matches == 0} {
                    send_msg "${designtask_count}-4" ERROR "Extraction of ${filename} mem location failed."
                }
                set slice_begin [get_property ram_slice_begin [get_cells $inst]]
                set slice_end [get_property ram_slice_end [get_cells $inst]]
                set slice_width [expr {$slice_end - $slice_begin + 1}]
                if {$slice_width < $fake_slice_width} {
                    set slice_end [expr {$slice_begin + $fake_slice_width - 1}]
                    set slice_width $fake_slice_width
                }
                set addr_begin [get_property ram_addr_begin [get_cells $inst]]
                set addr_end [get_property ram_addr_end [get_cells $inst]]
                set addr_end [expr {($addr_end + 1) * $addr_end_multiplier - 1}]
                set bit_layout [get_property "MEM.PORTA.DATA_BIT_LAYOUT" [get_cells $inst]]
                set read_width_a [get_property "READ_WIDTH_A" [get_cells $inst]]
                set read_width_b [get_property "READ_WIDTH_B" [get_cells $inst]]
                set slr_index [get_property "SLR_INDEX" [get_cells $inst]]
                if {$schema eq "Processor"} {
                    puts $fileout "        <BitLane MemType=\"$loc_prefix\" Placement=\"$loc_suffix\">"
                    puts $fileout "          <DataWidth MSB=\"$slice_end\" LSB=\"$slice_begin\"/>"
                    puts $fileout "          <AddressRange Begin=\"$addr_begin\" End=\"$addr_end\"/>"
                    puts $fileout "          <Parity ON=\"false\" NumBits=\"0\"/>"
                    puts $fileout "        </BitLane>"
                } else {
                    puts $fileout "      <BRAM MemType=\"$loc_prefix\" Placement=\"$loc_suffix\" Read_Width_A=\"$read_width_a\" Read_Width_B=\"$read_width_b\" SLR_INDEX=\"$slr_index\">"
                    puts $fileout "        <DataWidth_PortA MSB=\"$slice_end\" LSB=\"$slice_begin\"/>"
                    puts $fileout "        <AddressRange_PortA Begin=\"$addr_begin\" End=\"$addr_end\"/>"
                    puts $fileout "        <BitLayout_PortA pattern=\"$bit_layout\"/>"
                    puts $fileout "        <DataWidth_PortB MSB=\"0\" LSB=\"0\"/>"
                    puts $fileout "        <AddressRange_PortB Begin=\"0\" End=\"0\"/>"
                    puts $fileout "        <BitLayout_PortB pattern=\"\"/>"
                    puts $fileout "        <Parity ON=\"false\" NumBits=\"0\"/>"
                    puts $fileout "      </BRAM>"
                }
            }
            if {$schema eq "Processor"} {
                puts $fileout "      </BusBlock>"
                puts $fileout "    </AddressSpace>"
                puts $fileout "  </Processor>"
            } else {
                puts $fileout "    </MemoryLayout>"
                puts $fileout "  </MemoryArray>"
            }
        }
    }

    puts $fileout "  <Config>"
    puts $fileout "    <Option Name=\"Part\" Val=\"$part\"/>"
    puts $fileout "  </Config>"
    puts $fileout "</MemInfo>"
    close $fileout
    send_msg "${designtask_count}-4" INFO "MMI dumped to ${filepath}"
}

# Dump INIT_XX strings for the given BRAMs to an output file.
#
# In the output file, the BRAMs and their INIT_XX strings will be sorted in
# increasing order. This proc is a time-saver because the Vivado GUI's property
# viewer does not sort the INIT_XX strings numerically.
#
# Args:
#   filename:         Where to write
#   brams:            A list of BRAM cells.
#   designtask_count: A number used for logging with `send_msg`.
proc dump_init_strings {filename brams designtask_count} {
    # For each OTP BRAM, dump all the INIT_XX strings.
    send_msg "${designtask_count}-1" INFO "Dumping INIT_XX strings to ${filename}"

    set workroot [file dirname [info script]]
    set filepath "${workroot}/${filename}"
    set fileout [open $filepath "w"]

    foreach inst [lsort -dictionary $brams] {
        set bram [get_cells $inst]

        set loc [get_property LOC $bram]
        puts $fileout "LOC: $loc"

        set init_count 0
        while 1 {
            set key [format "INIT_%.2X" $init_count]
            if { [llength [list_property $bram $key]] eq 0 } {
                break
            }
            set val [get_property $key $bram]
            puts $fileout "$key $val"
            incr init_count
        }

        puts $fileout ""
    }
    close $fileout
    send_msg "${designtask_count}-4" INFO "INIT_XX strings dumped to ${filepath}"
}

set fpga_family [get_property FAMILY [get_parts [get_property PART [current_design]]]]

switch ${fpga_family} {
  kintex7 {
    set bram_regex "BMEM.*.*"
  }
  kintexu {
    set bram_regex "BLOCKRAM.*.*"
  }
  default {
    set bram_regex "BMEM.*.*"
  }
}
set mem_type_regex {(RAMB\d+)_(\w+)}

set gen_mem_info {{brams mem_type_regex fake_word_width addr_end_multiplier schema} {
  dict set mem_info brams $brams
  dict set mem_info mem_type_regex $mem_type_regex
  dict set mem_info fake_word_width $fake_word_width
  dict set mem_info addr_end_multiplier $addr_end_multiplier
  return [dict set mem_info schema $schema]
}}

# The scrambled Boot ROM is actually 39 bits wide, but we need to pretend that
# it's 40 bits, or else we will be unable to encode our ROM data in a MEM file
# that updatemem will understand.
#
# Suppose we did not pad the width, leaving it at 39 bits. Now, if we encode a
# word as a 10-digit hex string, updatemem would splice an additional zero bit
# into the bitstream because each hex digit is strictly 4 bits. If we wrote four
# words at a time, as a 39-digit hex string (39*4 is nicely divisible by 4),
# updatemem would fail to parse the hex string, saying something like "Data
# segment starting at 0x00000000, has exceeded data limits." The longest hex
# string it will accept is 16 digits, or 64 bits.
#
# A hack that works is to pretend the data width is actually 40 bits. Updatemem
# seems to write that extra zero bit into the ether without complaint.
set rom_brams [split [get_cells -hierarchical -filter " PRIMITIVE_TYPE =~ ${bram_regex} && NAME =~ *u_rom_ctrl*"] " "]
dict set memInfo rom [apply $gen_mem_info $rom_brams $mem_type_regex 40 1 "Processor"]

# OTP does not require faking the word width, but it has its own quirk. It seems
# each 22-bit OTP word is followed by 15 zero words. The MMI's <AddressSpace>
# and <AddressRange> tags need to account for this or else updatemem will think
# that its data input overruns the address space. The workaround is to pretend
# the address space is 16 times larger than we would normally compute.
set otp_brams [split [get_cells -hierarchical -filter " PRIMITIVE_TYPE =~ ${bram_regex} && NAME =~ *u_otp_macro*"] " "]
dict set memInfo otp [apply $gen_mem_info $otp_brams $mem_type_regex 0 16 "Processor"]

# The flash banks have 76-bit wide words. 64 bits are data, and 12 bits are metadata / integrity.
for {set bank 0} {$bank < 2} {incr bank} {
  for {set partition 0} {$partition < 3} {incr partition} {
    set flash_info_brams [split [get_cells -hierarchical -filter " PRIMITIVE_TYPE =~ ${bram_regex} && NAME =~ *u_flash_ctrl*gen_prim_flash_banks[${bank}]*gen_info_types[${partition}].u_info_mem*gen_xpm.gen_split[0].*"] " "]
    dict set memInfo "flash${bank}_info${partition}_data" [apply $gen_mem_info $flash_info_brams $mem_type_regex 0 1 "MemoryArray"]
    set flash_info_brams [split [get_cells -hierarchical -filter " PRIMITIVE_TYPE =~ ${bram_regex} && NAME =~ *u_flash_ctrl*gen_prim_flash_banks[${bank}]*gen_info_types[${partition}].u_info_mem*gen_xpm.gen_split[64].*"] " "]
    dict set memInfo "flash${bank}_info${partition}_intg" [apply $gen_mem_info $flash_info_brams $mem_type_regex 0 1 "MemoryArray"]
  }
}

generate_mmi "memories.mmi" $memInfo 1

# For debugging purposes, dump the INIT_XX strings for ROM and OTP.
dump_init_strings "rom_init_strings.txt" $rom_brams 3
dump_init_strings "otp_init_strings.txt" $otp_brams 4
