module packet_parser (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  byte_in,
    input  wire        byte_valid,

    output reg         packet_valid,
    output reg [7:0]   msg_type,
    output reg [7:0]   symbol_id,
    output reg [31:0]  timestamp,
    output reg [31:0]  order_id,
    output reg         side,
    output reg [31:0]  price,
    output reg [23:0]  quantity
);

    // State machine states
    localparam IDLE       = 2'd0;
    localparam COLLECTING = 2'd1;
    localparam COMPLETE   = 2'd2;

    reg [1:0] state;
    reg [4:0] byte_count;        // counts 0-17, needs 5 bits (up to 31)
    reg [7:0] buffer [0:17];     // 18-byte buffer, one byte per index

    always @(posedge clk) begin
        if (rst) begin
            state        <= IDLE;
            byte_count   <= 0;
            packet_valid <= 0;
        end else begin

            // packet_valid should only pulse for ONE cycle, so default it low
            // every cycle unless we explicitly set it in the COMPLETE state
            packet_valid <= 0;

            case (state)

                IDLE: begin
                    if (byte_valid) begin
                        buffer[0]  <= byte_in;
                        byte_count <= 1;
                        state      <= COLLECTING;
                    end
                end

                COLLECTING: begin
                    if (byte_valid) begin
                        buffer[byte_count] <= byte_in;
                        if (byte_count == 17) begin
                            state <= COMPLETE;
                        end else begin
                            byte_count <= byte_count + 1;
                        end
                    end
                end

                COMPLETE: begin
                    // Decode the 18-byte buffer into named fields.
                    // Big-endian: most significant byte first.
                    msg_type     <= buffer[0];
                    symbol_id    <= buffer[1];
                    timestamp    <= {buffer[2], buffer[3], buffer[4], buffer[5]};
                    order_id     <= {buffer[6], buffer[7], buffer[8], buffer[9]};
                    side         <= buffer[10][0];  // only bit 0 matters (0 or 1)
                    price        <= {buffer[11], buffer[12], buffer[13], buffer[14]};
                    quantity     <= {buffer[15], buffer[16], buffer[17]};

                    packet_valid <= 1;
                    byte_count   <= 0;
                    state        <= IDLE;
                end

            endcase
        end
    end

endmodule
