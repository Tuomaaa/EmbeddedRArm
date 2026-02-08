module Servo_PWM(
    input clk,
    input reset,
    input [7:0] angle,
    output wire pwm_out
);

    reg [17:0] counter;  // 18位足够计到 239999
    
    wire [7:0] angle_clamped = (angle > 180) ? 180 : angle;
    

    wire [17:0] pulse_width = 6000 + angle_clamped * 133;
    

    always @(posedge clk or posedge reset) begin
        if (reset)
            counter <= 0;
        else begin
            if (counter == 239999)
                counter <= 0;
            else
                counter <= counter + 1;
        end
    end
    
    assign pwm_out = (counter < pulse_width) ? 1 : 0;

endmodule