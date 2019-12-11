`ifndef __REGISTERS__
`define __REGISTERS__

`define CMD_DIO_WRITE 8'h00
`define CMD_DIO_READ 8'h01
`define CMD_DIO_TRIS 8'h02
`define CMD_PERIPHERAL_WRITE 8'h03
`define CMD_PERIPHERAL_READ 8'h04
`define CMD_DELAY 8'h05
`define CMD_PWM_ON_PERIOD 8'h06
`define CMD_PWM_OFF_PERIOD 8'h07
`define CMD_ADC_READ 8'h08
`define CMD_LA_START 8'h09
`define CMD_LA_STOP 8'h0A
`define CMD_REGISTER_SET_POINTER 8'h0B
`define CMD_REGISTER_WRITE 8'h0C
`define CMD_REGISTER_READ 8'h0D
//`define CMD_DAC_WRITE 8'h0E
`define CMD_LA_RESET 8'h0E
`define CMD_SM_HALT 8'h0F
//TODO: adjust for more commands: frequency measurement etc

`define REG_BPIO_OE config_register[4'h0][BP_PINS-1:0]

`define REG_BPIO_OD config_register[4'h1][BP_PINS-1:0]

`define REG_HW_CONFIG config_register[4'h2]
`define REG_HW_CONFIG_PULLUPS_EN config_register[4'h2][0]

/*`define REG_LA_CONFIG config_register[4'h3]
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
*/
`define REG_ADC_CALIBRATE config_register[4'hA][3]
`define REG_ADC_CLOCK_DIVIDER config_register[4'hA][2:0]

`define REG_PERIPHERAL_0 config_register[4'hC]
//`REG_PERIPHERAL_0[0] .cpol(1'b1), //cpol,				// clock polarity
//`REG_PERIPHERAL_0[1] .cpha(1'b0), //cpha,				// clock phase
`define REG_PERIPHERAL_1 config_register[4'hD]
`define REG_PERIPHERAL_2 config_register[4'hE]
`define REG_PERIPHERAL_3 config_register[4'hF]



`endif
