`timescale 1ns / 1ps
module FT4052C(
    input wire CLK,
    
    input wire [31:0] INTEGRATION,
    input wire TRIGGER,
    
    output wire CR,
    output wire VA,
    
    output wire SSC,
    output wire A1,
    output wire A2,
    output wire A3,
    output wire A4,
    
    output wire C3,
    output wire C1,
    output wire C2,
    output wire SG,
    output wire RG,
    
    output wire HDR_ENABLE,
    output wire HDR_R,
    
    output wire HDR_ADC_W,
    
    output wire ADC_CLK,
    output wire ADC_CDS1,
    output wire ADC_CDS2,
    output wire ADC_INIT,
    
    output wire DRV_MAX,
    output wire DRV_V11,
    output wire DRV_TG,
    output wire DRV_LVL,
    
    // params
    output wire [15:0] PARAM_X,
    output wire [15:0] PARAM_Y,
    output wire [15:0] PARAM_ACTIVE_X,
    output wire [15:0] PARAM_ACTIVE_Y,
    output wire [15:0] PARAM_OFFSET_X,
    output wire [15:0] PARAM_OFFSET_Y,

    output wire SHUTTER_ON,
    output wire SHUTTER_OFF,
    
    output wire [2:0] _state
    //output wire [31:0] _counter,
    //output wire [12:0] _x, 
    //output wire [12:0] _y 
    );
    
    // width = 4763 cols
    // height = 5356 rows
    // view 4271 * 5356 = 22875476, 45750952 (*2), 45750968 (+16), 2BA1AB8
    // INTEGRATION = time of integration in microseconds
    
    // finite state machine
    reg [2:0] state; // 0-idle, 1-shutter on, 2-reset, 3-integration, 4-shutter off, 5-readout
    reg [5:0] microsecond;
    reg [31:0] counter;
    reg [9:0] reset;
    reg [27:0] period_on;
    reg [27:0] period_off;
     
    wire state_idle = state == 3'b000;
    wire state_on = state == 3'b001;
    wire state_reset = state == 3'b010;
    wire state_integration = state == 3'b011;
    wire state_off = state == 3'b100;
    wire state_readout = state == 3'b101;
    
    // phaser (divide clock to 6 phases [0..5])
    reg [5:0] phaser;
    
    // pixel phases
    wire c3 = phaser[2] || phaser[3] || phaser[4];     
    wire c1 = phaser[0] || phaser[4] || phaser[5];     
    wire c2 = phaser[0] || phaser[1] || phaser[2]; 
    wire sg = phaser[2] || phaser[3] || phaser[4]; 
    wire rg = phaser[2];
    
    // horizontal
    reg [12:0] x;
    wire x_32 = x < 32;
    wire x_124 = x < 124;
    wire x_186 = x < 186;
    wire x_248 = x < 248;
    wire x_310 = x < 310;
    wire x_372 = x < 372;
    wire x_434 = x < 434;
    wire x_496 = x < 496;
    wire x_558 = x < 558;
    wire x_684 = x < 684;
    wire x_692_n = x >= 692;
    
    wire a1 = !x_186 &&  x_496  &&  x_684;  //186,496
    wire a2 = (x_124 || !x_310) || !x_684;  //124,310
    wire a3 = (x_248 || !x_434) || !x_684;  //248,434
    wire a4 = (x_372 || !x_558) || !x_684;  //372,558
    wire ssc = x_684;
    
    // vertical
    reg [12:0] y;
    wire y_0 = y == 0;
    
    
    // finite state machine
    always @ (posedge CLK)
    begin
        case (state)
        0:begin
            // idle
            if (TRIGGER == 1'b1)
            begin
                state <= 1; // to shutter on
                counter <= INTEGRATION;
                period_on <= 24000000; // shutter on, 4 800 000 clocks (100ms) 
            end   
        end
        1:begin
            // shutter on
            if (period_on == 0)
            begin
                state <= 2; // to reset             
                reset <= 960; // reset, 960 clocks (20 microseconds) 
            end else begin
                period_on <= period_on - 1;            
            end            
        end
        2:begin
            // reset (min 10 microseconds)
            if (reset == 0)
            begin
                state <= 3; // to integration             
                microsecond <= 0;
            end else begin
                reset <= reset - 1;            
            end            
        end
        3:begin
            // integration
            if (microsecond == 47)
            begin
                microsecond <= 0;
                counter <= counter - 1;
                if (counter == 0)
                begin
                    state <= 4;            
                    period_off <= 24000000; // shutter off, 4 800 000 clocks (100ms) 
                end  
            end else begin
                microsecond <= microsecond + 1;            
            end
        end
        4:begin
            // shutter off
            if (period_off == 0)
            begin
                phaser <= 6'b000001;
                y <= 0;
                x <= 0;                
                state <= 5; // to readout             
            end else begin
                period_off <= period_off - 1;            
            end            
        end
        5:begin
            // readout
            phaser <= {phaser[4:0], phaser[5]};
            
            if (phaser[5]) 
            begin
                if (x == 4762)
                begin
                    x <= 0;
                    y <= y + 1;            
                end else begin
                    x <= x + 1;    
                end        
            end
            
            if (y == 5355 & x == 4762 & phaser[5])
                state <= 0;
        end
        endcase
    end
    
    
    // final calcs
    reg cr_reg;
    reg va_reg;
    reg ssc_reg;
    reg a1_reg;
    reg a2_reg;
    reg a3_reg;
    reg a4_reg;
    reg c3_reg;
    reg c1_reg;
    reg c2_reg;
    reg rg_reg;
    reg sg_reg;
    reg hdr_enable_reg;
    reg hdr_r_reg;
    reg hdr_adc_w_reg;
    reg adc_clk_reg;
    reg adc_cds1_reg;
    reg adc_cds2_reg;
    reg adc_init_reg;
    reg drv_max_reg;
    reg drv_tg_reg;
    reg drv_v11_reg;
    reg drv_lvl_reg;
    reg shutter_on_reg;
    reg shutter_off_reg;
    
    always @ (posedge CLK)
    begin
        cr_reg <= state_reset;
        va_reg <= state_readout;
        ssc_reg <= ssc;
        a1_reg <= a1 & state_readout;
        a2_reg <= ~(state_idle | state_reset) & (state_integration | a2);
        a3_reg <= ~(state_idle | state_reset) & (state_integration | a3);
        a4_reg <= ~(state_idle | state_reset) & (state_integration | a4);
        c3_reg <= c3 & ~ssc;
        c1_reg <= c1 | ssc;
        c2_reg <= c2 | ssc;
        sg_reg <= sg;
        rg_reg <= rg;
        adc_init_reg <= state_readout & y_0;    
    end
    
    // header
    always @ (posedge CLK)
    begin
        hdr_enable_reg <= state_readout & y_0 & x_32;
        hdr_r_reg <= state_readout & y_0 & x_32 & phaser[1]; 
        hdr_adc_w_reg <= state_readout & ((y_0 & x_32 & phaser[3]) | (x_692_n & (phaser[1] | phaser[4]))); //todo 4 pixels shift 
    end
    
    // ADC
    always @ (posedge CLK)
    begin
        adc_clk_reg <= state_readout & (phaser[0] | phaser[1] | phaser[5]);
        adc_cds1_reg <= state_readout & (phaser[2]);
    end
    always @ (negedge CLK)
    begin
        adc_cds2_reg <= state_readout & (phaser[0] | phaser[5]);
    end
    
    // header
    always @ (posedge CLK)
    begin
        drv_max_reg <= ~(state_on | state_reset | state_integration | state_off | state_readout);
        drv_tg_reg <= state_readout;
        drv_v11_reg <= ~(state_readout);
        drv_lvl_reg <= ~(state_integration | state_off | state_readout);
    end
    
    // shutter
    always @ (posedge CLK)
    begin
        shutter_on_reg <= state_on;
        shutter_off_reg <= state_off;
    end
    
    
    // assignes
    assign CR = cr_reg;
    assign VA = va_reg;
    
    assign SSC = ssc_reg;
    assign A1 = a1_reg;
    assign A2 = a2_reg;
    assign A3 = a3_reg;
    assign A4 = a4_reg;
    
    assign C3 = c3_reg;
    assign C1 = c1_reg;
    assign C2 = c2_reg;
    assign SG = sg_reg;
    assign RG = rg_reg;
    
    assign HDR_ENABLE = hdr_enable_reg;
    assign HDR_R = hdr_r_reg;
    
    assign HDR_ADC_W = hdr_adc_w_reg;
    
    assign ADC_CLK = adc_clk_reg;
    assign ADC_CDS1 = adc_cds1_reg;
    assign ADC_CDS2 = adc_cds2_reg;
    assign ADC_INIT = adc_init_reg;
    
    assign DRV_MAX = drv_max_reg;
    assign DRV_V11 = drv_v11_reg;
    assign DRV_TG = drv_tg_reg;
    assign DRV_LVL = drv_lvl_reg;
    
    // params
    assign PARAM_X = 4071;
    assign PARAM_Y = 5356; 
    assign PARAM_ACTIVE_X = 4008;
    assign PARAM_ACTIVE_Y = 5344;
    assign PARAM_OFFSET_X = 48;
    assign PARAM_OFFSET_Y = 6; 

    // shutter
    assign SHUTTER_ON = shutter_on_reg;
    assign SHUTTER_OFF = shutter_off_reg;
    
    // temporary
    assign _state = state;
    //assign _counter = counter;
    //assign _x = x;
    //assign _y = y;
    
initial 
begin
    phaser <= 6'b000001;
    x <= 0;
    y <= 0;
    state <= 0;
    microsecond <= 0;
    counter <= 0;
    reset <= 0;
    
    hdr_adc_w_reg <= 0;
end    


endmodule
