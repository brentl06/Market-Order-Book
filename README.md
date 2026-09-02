# Market-Order-Book
## Project Overview
Verilog implementation of a market data feed handler and order book. These are crucial pieces of trading infrastructure that operate when orders are placed, deciding using onboard logic whether to fill, rest, or cancel an order. Synthesized and routed on the Diligent Nexys-A7 100T FPGA using Xilinx Vivado. Primary modules in this project are `packet_parser.v`, `order_book.v`, and `market_feed_top.v`.

## 18-Byte Binary Wire Protocol
| Bytes | Output Port | Width | Description |
| -------- | -------- | -------- | -------- |
| 0 | msg_type | 8b | 0x01 is ADD, 0x02 is CANCEL, 0x03 is EXECUTE |
| 1 | symbol_id | 8b | numerical substitute for a traditional ticker symbol |
| 2-5 | timestamp | 32b | time the order is placed, used as tie-breaker |
| 6-9 | order_id | 32b | value assigned to each placed order |
| 10 | side | 1b | 0 is buy, 1 is sell |
| 11-14 | price | 32b | price of the order |
| 15-17 | quantity | 24b | quantity of the order |

## Order Book
`order_book.v` holds 64 slots for resting orders, where each slot is occupied by the output port values from the 18-byte wire.
The order book originally scanned all 64 slots combinationally, though this quickly resulted in timing issues during synthesis and implementation. Changed to "chunk reading", where 8 slots are read and compared to find the order with price-time priority.
