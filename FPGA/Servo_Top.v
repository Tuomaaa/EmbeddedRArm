module Servo_Top(
    input clk,
    input rx,
    output pwm_out,
    output reg led_debug,
    output reg led_debug2
);

    wire [7:0] uart_data;
    wire data_ready;
    reg [7:0] angle = 90;
    wire pwm_signal;

    // 直接用UART，不经过decoder
    UART uart_inst(
        .clk(clk),
        .rx(rx),
        .out(uart_data),
        .data_ready(data_ready),
        .led_debug(led_debug),
        .led_debug2(led_debug2)
    );
    

    // 收到任何字节就更新angle
    always @(posedge clk) begin
        if (data_ready)
            angle <= uart_data;
    end

    Servo_PWM servo1(
        .clk(clk),
        .reset(1'b0),
        .angle(angle),
        .pwm_out(pwm_signal)
    );

    assign pwm_out = pwm_signal;

endmodule