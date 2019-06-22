`timescale 1ms / 1ns
module adc_spi (
    input wire CLK,
    input wire ENABLE,
    input wire [5:0] GAIN,
    input wire [8:0] OFFSET,
    output reg  ADC_SLOAD,
    output wire ADC_SCLK,
    output reg  ADC_SDATA
);

reg [51:0] load;
reg [51:0] data;

always @ (posedge CLK)
begin
    if(ENABLE == 1'b0) 
    begin	
        ADC_SLOAD <= 1'b1;
        ADC_SDATA <= 1'b0;
        load <= {52'b1000000000000000010000000000000000100000000000000001};
        data <= {1'b0, OFFSET[0], OFFSET[1], OFFSET[2], OFFSET[3], OFFSET[4], OFFSET[5], OFFSET[6], OFFSET[7], OFFSET[8], 8'b00010100, GAIN[0], GAIN[1], GAIN[2], GAIN[3], GAIN[4], GAIN[5], 28'b0000000100000001010000000000};
    end else begin
        ADC_SLOAD <= load[0];
        ADC_SDATA <= data[0];
        load <= {1'b1, load[51:1]};
        data <= {1'b0, data[51:1]};
	end
end

assign ADC_SCLK = ~CLK;

initial
begin
    ADC_SLOAD <= 1'b1;
    ADC_SDATA <= 1'b0;
    load <= 52'b1111111111111111111111111111111111111111111111111111;
    data <= 52'b0;
end


endmodule


/*


reg [51:0] load;
reg [51:0] data;
reg [5:0] p;

always @ (posedge CLK)
begin
    if(ENABLE == 1'b0) 
    begin	
        p <= 0; 
        ADC_SLOAD <= 1'b1;
        ADC_SDATA <= 1'b0;
        load <= {52'b1000000000000000010000000000000000100000000000000001};
        data <= {1'b0, OFFSET[0], OFFSET[1], OFFSET[2], OFFSET[3], OFFSET[4], OFFSET[5], OFFSET[6], OFFSET[7], OFFSET[8], 8'b00010100, GAIN[0], GAIN[1], GAIN[2], GAIN[3], GAIN[4], GAIN[5], 28'b0000000100000001010000000000};
    end else begin
        ADC_SLOAD <= load[p];
        ADC_SDATA <= data[p];
        p <= (p == 51) ? 51 : p + 1;
	end
end

assign ADC_SCLK = ~CLK;

initial
begin
    p <= 0;
    ADC_SLOAD <= 1'b1;
    ADC_SDATA <= 1'b0;
    load <= 0;
    data <= 0;
end


*/



