module UART(
input clk,
input rx,
output reg[7:0] out,
output reg data_ready,
output reg led_debug=0,
output reg led_debug2=1
);

parameter CLKBIT = 12000000/115200; //12MHZ sys clk 11520 baud rate (104 but set to be compatible for 9600 baud rate as well)

reg [10:0] clkcnt;
reg[7:0] data;
reg[3:0] idx = 0;

localparam IDLE = 2'b00;
localparam START = 2'b01;
localparam STOP = 2'b10;
localparam DATA = 2'b11;

reg[1:0] STATE=IDLE;




always@(posedge clk)begin
    case(STATE)

        IDLE: begin
            data_ready <=0;
            clkcnt <= 0;
            
            led_debug2 <= 1; // 在 IDLE 状态时点亮 LED
            if(rx == 0)
                ///led_debug2 <= 0; // 在 START 状态时熄灭 LED

                STATE <= START;
            

        end

        START: begin
            led_debug2 <= 0; // 在 START 状态时熄灭 LED
            if (clkcnt == CLKBIT/2) begin  
                clkcnt <= 0;
                if (rx == 0)begin
                    idx <= 0;        
                    STATE <= DATA;
                    end
                else
                    STATE <= IDLE;  
            end else
                clkcnt <= clkcnt + 1;

        end

        DATA: begin

            if(clkcnt == CLKBIT) begin
                clkcnt <= 0;
                data[idx] <= rx;
                if(idx == 7)
                    STATE<=STOP;
                else
                    idx <= idx + 1;
            end else
                clkcnt <= clkcnt + 1;
        end
/*
        STOP: begin
            if (clkcnt == CLKBIT) begin
                clkcnt <= 0;
                if (rx == 1) begin       
                    out <= data;
                    data_ready <= 1;
                end
                STATE <= IDLE;
            end else
                clkcnt <= clkcnt + 1;
        end
    endcase
*/
        STOP: begin

            if (clkcnt == CLKBIT) begin
                clkcnt <= 0;
                // --- 删除或注释掉下面这行 ---
                // if (rx == 1) begin 
                // -------------------------
                    out <= data;     // 无条件接收数据
                    data_ready <= 1;
                // --- 删除或注释掉下面这行 ---
                // end
                // -------------------------
                STATE <= IDLE;
            end else
                clkcnt <= clkcnt + 1;
        end
    endcase


end







endmodule