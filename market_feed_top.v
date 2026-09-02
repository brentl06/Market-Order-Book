module market_feed_top (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] byte_in,
    input  wire       byte_valid,

    output wire       op_done,
    output wire       op_success,
    output wire [5:0] last_slot
);

    // Wires connecting parser output -> order book input
    wire        packet_valid;
    wire [7:0]  msg_type;
    wire [7:0]  symbol_id;
    wire [31:0] timestamp;   // consumed by order_book for price-time crossing priority
    wire [31:0] order_id;
    wire        side;
    wire [31:0] price;
    wire [23:0] quantity;

    packet_parser parser_inst (
        .clk(clk),
        .rst(rst),
        .byte_in(byte_in),
        .byte_valid(byte_valid),
        .packet_valid(packet_valid),
        .msg_type(msg_type),
        .symbol_id(symbol_id),
        .timestamp(timestamp),
        .order_id(order_id),
        .side(side),
        .price(price),
        .quantity(quantity)
    );

    order_book book_inst (
        .clk(clk),
        .rst(rst),
        .packet_valid(packet_valid),
        .msg_type(msg_type),
        .symbol_id(symbol_id),
        .timestamp(timestamp),
        .order_id(order_id),
        .side(side),
        .price(price),
        .quantity(quantity),
        .op_done(op_done),
        .op_success(op_success),
        .last_slot(last_slot)
    );

endmodule