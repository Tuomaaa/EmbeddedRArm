module AngleSweep(
    input clk,
    input reset,
    output reg [7:0] angle,
    output reg direction      // 0 = 增加, 1 = 减少
);
    parameter SPEED_DIV = 120000;  // 加这行！约100ms换一度

    reg [19:0] speed_cnt;

    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            angle <= 8'd0;
            direction <= 1'b0;
            speed_cnt <= 0;
        end
        else begin
            if (speed_cnt == SPEED_DIV - 1) begin
                speed_cnt <= 0;
                
                if (direction == 1'b0) begin
                    // 正在增加
                    if (angle == 8'd180) begin
                        direction <= 1'b1;
                        angle <= 8'd179;
                    end
                    else begin
                        angle <= angle + 1;
                    end
                end
                else begin
                    // 正在减少
                    if (angle == 8'd0) begin
                        direction <= 1'b0;
                        angle <= 8'd1;
                    end
                    else begin
                        angle <= angle - 1;
                    end
                end
            end
            else begin
                speed_cnt <= speed_cnt + 1;
            end
        end
    end

endmodule