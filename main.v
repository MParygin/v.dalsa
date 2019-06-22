`timescale 1ns / 1ps
module main(

    //------------------ Cypress CY7C68013A-56
     input wire      fx2Clk_in,
    output wire[1:0] fx2Addr_out,   // select FIFO: "10" for EP6OUT, "11" for EP8IN
     inout wire[7:0] fx2Data_io,    // 8-bit data to/from FX2
    // When EP6OUT selected:
    output wire      fx2Read_out,   // asserted (active-low) when reading from FX2
    output wire      fx2OE_out,     // asserted (active-low) to tell FX2 to drive bus
     input wire      fx2GotData_in, // asserted (active-high) when FX2 has data for us
    // When EP8IN selected:
    output wire      fx2Write_out,  // asserted (active-low) when writing to FX2
     input wire      fx2GotRoom_in, // asserted (active-high) when FX2 has room for more data from us
    output wire      fx2PktEnd_out, // asserted (active-low) when a host read needs to be committed early


    //------------------ ADC AD9826
    output wire      ADC_SLOAD,
    output wire      ADC_SCLK,
    output wire      ADC_SDATA,
    output wire      ADC_CLK,
    output wire      ADC_CDS1,
    output wire      ADC_CDS2,
     input wire[7:0] ADC_DATA,

    // CCD
    output wire CCD_CR,
    output wire CCD_RG,
    output wire CCD_SG,
    output wire CCD_A1T,
    output wire CCD_A1B,
    output wire CCD_A2T,
    output wire CCD_A2B,
    output wire CCD_A3T,
    output wire CCD_A3B,
    output wire CCD_A4T,
    output wire CCD_A4B,
    output wire CCD_TGW,
    output wire CCD_TGX,
    output wire CCD_TGY,
    output wire CCD_TGZ,
    output wire CCD_C1W,
    output wire CCD_C1X,
    output wire CCD_C2W,
    output wire CCD_C2X,
    output wire CCD_C3W,
    output wire CCD_C3X,

    // DRV
    output wire DRV_MAX,
    output wire DRV_V11,
    output wire DRV_TG,
    output wire DRV_LVL,
    
    output wire POWER_ON,

    output wire SHUTTERon,
    output wire SHUTTERoff,

    output wire LED_GREEN,
    output wire LED_RED
);
    // global buffer clock
    wire CLK_48;
    IBUFG clk_48_inst(.O(CLK_48),.I(fx2Clk_in));

    // global clock divider for non critical purposes
    reg [2:0] clock_reg;
    wire clk_6 = clock_reg[2];
    always @ (posedge CLK_48)
    begin
        clock_reg <= clock_reg + 1;    
    end

    // global settings
    reg [8:0] tune_offset;
    reg [5:0] tune_gain;
    reg [31:0] tune_integration;
    
    // global values
    wire [15:0] param_x, param_y, param_active_x, param_active_y, param_offset_x, param_offset_y;


    // connections
    wire hdr_enable;
    wire hdr_r;
    wire hdr_adc_w;
    reg [4:0] hdr_pos;
    reg [7:0] hdr_value;
    wire [7:0] hdr_adc = (hdr_enable) ? hdr_value : ADC_DATA;
    always @ (posedge CLK_48)
    begin
        // write header byte value
        if (hdr_enable & hdr_adc_w)
            hdr_pos <= hdr_pos + 1;
        // select header value    
        if (hdr_enable & hdr_r)
        begin
            case (hdr_pos)
            0:  hdr_value <= 8'h41;
            1:  hdr_value <= 8'h43;
            2:  hdr_value <= 8'h4D;
            3:  hdr_value <= 8'h50;

            4:  hdr_value <= param_x[7:0];
            5:  hdr_value <= param_x[15:8];
            6:  hdr_value <= param_y[7:0];
            7:  hdr_value <= param_y[15:8];
            
            8:  hdr_value <= param_active_x[7:0];
            9:  hdr_value <= param_active_x[15:8];
            10:  hdr_value <= param_active_y[7:0];
            11:  hdr_value <= param_active_y[15:8];
            
            12:  hdr_value <= param_offset_x[7:0];
            13:  hdr_value <= param_offset_x[15:8];
            14:  hdr_value <= param_offset_y[7:0];
            15:  hdr_value <= param_offset_y[15:8];
            
            16:  hdr_value <= 8'h00;
            17:  hdr_value <= {2'b00, tune_gain[5:0]};
            18:  hdr_value <= tune_offset[7:0];
            19:  hdr_value <= {7'b0000000, tune_offset[8]};
            
            20:  hdr_value <= tune_integration[7:0];
            21:  hdr_value <= tune_integration[15:8];
            22:  hdr_value <= tune_integration[23:16];
            23:  hdr_value <= tune_integration[31:24];
            
            24:  hdr_value <= 8'h20;
            25:  hdr_value <= 8'h20;
            26:  hdr_value <= 8'h20;
            27:  hdr_value <= 8'h20;

            28:  hdr_value <= 8'h20;
            29:  hdr_value <= 8'h20;
            30:  hdr_value <= 8'h20;
            31:  hdr_value <= 8'h20;

            endcase        
        end    
    end 

  
    wire [6:0] channel; 
    wire [7:0] h2f;
    wire h2f_v;
    reg [7:0] control;
    
    always @ (posedge CLK_48)
    begin
        if (h2f_v == 1'b1)
        begin
            case (channel)
            0: control <= h2f;
            1: tune_gain <= h2f[5:0];
            2: tune_offset[7:0] <= h2f;
            3: tune_offset[8] <= h2f[0];
            4: tune_integration[7:0] <= h2f; 
            5: tune_integration[15:8] <= h2f; 
            6: tune_integration[23:16] <= h2f; 
            7: tune_integration[31:24] <= h2f; 
            endcase      
        end
    end

    // FIFO
    wire [7:0] out_data;
    wire out_valid;
    wire out_ready;
    wire fifo_full;
    fifo_usb fifo_inst(
        .clk(CLK_48),
        .din(hdr_adc),
        .wr_en(hdr_adc_w),
        .full(fifo_full),
        .dout(out_data),
        .rd_en(out_ready),
        .empty(out_valid)
    );


    // CommFPGA module
    wire fx2Read;
    assign fx2Read_out = fx2Read;
    assign fx2OE_out = fx2Read;
    assign fx2Addr_out[1] = 1'b1;  // Use EP6OUT/EP8IN, not EP2OUT/EP4IN.
    comm_fpga_fx2 comm_fpga_fx2(
        // FX2 interface
        .fx2Clk_in(CLK_48),
        .fx2FifoSel_out(fx2Addr_out[0]),
        .fx2Data_io(fx2Data_io),
        .fx2Read_out(fx2Read),
        .fx2GotData_in(fx2GotData_in),
        .fx2Write_out(fx2Write_out),
        .fx2GotRoom_in(fx2GotRoom_in),
        .fx2PktEnd_out(fx2PktEnd_out),
        
        // Channel read/write interface
        .chanAddr_out(channel),
        .h2fData_out(h2f),
        .h2fValid_out(h2f_v),
        .h2fReady_in(1'b1), // always ready
        .f2hData_in(out_data),
        .f2hValid_in(~out_valid),
        .f2hReady_out(out_ready)
    );

    // ADC init
    wire adc_init;
    adc_spi adc_spi_inst(
        .CLK(clk_6),
        .ENABLE(adc_init), 
        .GAIN(tune_gain),
        .OFFSET(tune_offset), 
        .ADC_SLOAD(ADC_SLOAD), 
        .ADC_SCLK(ADC_SCLK), 
        .ADC_SDATA(ADC_SDATA)
    );

    // FT5066C timegen
    wire a1, a2, a3, a4;
    wire c1, c2, c3;
    wire va;
    wire shutter_on, shutter_off;
    
    FT4052C ft_inst(
    //FT5066C ft_inst(
        .CLK(CLK_48),
        .INTEGRATION(tune_integration),
        .TRIGGER(control[7]), //todo
        .CR(CCD_CR),
        .VA(va),
        .A1(a1), .A2(a2), .A3(a3), .A4(a4),
        .C3(c3), .C1(c1), .C2(c2),
        .SG(CCD_SG),.RG(CCD_RG),
        .HDR_ENABLE(hdr_enable), .HDR_R(hdr_r), .HDR_ADC_W(hdr_adc_w),
        .ADC_CLK(ADC_CLK), .ADC_CDS1(ADC_CDS1), .ADC_CDS2(ADC_CDS2), .ADC_INIT(adc_init),
        .DRV_MAX(DRV_MAX), .DRV_V11(DRV_V11), .DRV_TG(DRV_TG), .DRV_LVL(DRV_LVL),
        .PARAM_X(param_x), .PARAM_Y(param_y),
        .PARAM_ACTIVE_X(param_active_x), .PARAM_ACTIVE_Y(param_active_y),
        .PARAM_OFFSET_X(param_offset_x), .PARAM_OFFSET_Y(param_offset_y),
        .SHUTTER_ON(shutter_on), .SHUTTER_OFF(shutter_off)
    );
    
    assign CCD_TGW = a1;
    assign CCD_TGX = a1;
    assign CCD_TGY = a1;
    assign CCD_TGZ = a1;
    assign CCD_A1T = a1;
    assign CCD_A1B = a1;
    assign CCD_A2T = a2;
    assign CCD_A2B = a2;
    assign CCD_A3T = a3;
    assign CCD_A3B = a3;
    assign CCD_A4T = a4;
    assign CCD_A4B = a4;
    assign CCD_C1W = c1;
    assign CCD_C1X = c1;
    assign CCD_C2W = c2;
    assign CCD_C2X = c2;
    assign CCD_C3W = c3;
    assign CCD_C3X = c3;

    assign POWER_ON = 1'b1;
    
    assign SHUTTERon = shutter_on;
    assign SHUTTERoff = shutter_off;
    

    // tmp
    assign LED_GREEN = shutter_on;//fifo_full;
    assign LED_RED = shutter_off;

initial
begin
    tune_offset <= 10;
    tune_gain <= 0;
    tune_integration <= 1000000;
    hdr_value <= 8'h33;
    hdr_pos <= 0;
end

endmodule
