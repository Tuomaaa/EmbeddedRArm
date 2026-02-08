module UART_deco(
    input rx,
    input clk,
    output reg[2:0] motor,
    output reg[7:0] angle
);

    wire data_ready;
    wire [7:0] data_out;

    reg[1:0] STATE;
    localparam IDLE_M = 2'b00;
    localparam MOTOR = 2'b01;
    localparam IDLE_A = 2'b10;
    localparam ANGLE = 2'b11;
    wire led;
    UART uart_inst(
        .clk(clk),
        .rx(rx),
        .out(data_out),
        .data_ready(data_ready),
        .led_debug(led),
        .led_debug2()
    );
    always@(posedge clk) begin

        case(STATE)
            IDLE_M: begin
                if(data_ready == 0) begin
                    STATE <= MOTOR;
                end
            end
            MOTOR: begin
                if(data_ready) begin
                    motor <= data_out[2:0];
                    STATE <= IDLE_A;
                end
            end
            IDLE_A: begin
                if(data_ready == 0) begin
                    STATE <= ANGLE;
                end
            end
            ANGLE: begin
                if(data_ready) begin
                    angle <= data_out;
                    STATE <= IDLE_M;
                end
            end
        endcase

    end

endmodule