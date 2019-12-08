`ifndef __REGISTERS__
`define __REGISTERS__

`define REG_BPIO_OE config_register[4'h0][BP_PINS-1:0]

`define REG_BPIO_OD config_register[4'h1][BP_PINS-1:0]

`define REG_HW_CONFIG config_register[4'h2]
`define REG_HW_CONFIG_PULLUPS_EN config_register[4'h2][0]

`define REG_LA_CONFIG config_register[4'h3]
`define reg_la_io_quad config_register[4'h3][0]
`define reg_la_io_quad_direction config_register[4'h3][1]
`define reg_la_io_spi config_register[4'h3][2]
`define reg_la_clear_sample_counter config_register[4'h3][3]
`define reg_la_active config_register[4'h3][5]
`define reg_la_max_samples_reached config_register[4'h3][6]
`define reg_bpsm_reset config_register[4'h3][7]

`define reg_la_io_cs0 config_register[4'h3][8] //reserve upper bits for more SRAMs
`define reg_la_io_cs1 config_register[4'h3][9]

`define reg_la_sample_count rreg[4'h4]

`define REG_ADC_CALIBRATE config_register[4'hA][0]

`define REG_PERIPHERAL_0 config_register[4'hC]
`define REG_PERIPHERAL_1 config_register[4'hD]
`define REG_PERIPHERAL_2 config_register[4'hE]
`define REG_PERIPHERAL_3 config_register[4'hF]



`endif
