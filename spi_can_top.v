// ==============================================================================
// File: spi_can_top.v
// Standard: Verilog-2001
// Description: SPI to CAN converter for FPGA Prototype / SKY130 ASIC Mapping
// ==============================================================================

module spi_can_top (
    input  wire clk50,        // 50 MHz system clock
    input  wire rst_n,        // Active-low asynchronous reset
    input  wire spi_en,       // Enable for the SPI Master
    input  wire miso,         // SPI Master In Slave Out
    output wire mosi,         // SPI Master Out Slave In
    output wire sclk,         // SPI Clock (5 MHz)
    output wire cs_n,         // SPI Chip Select (Active Low)
    output wire can_h_gpio,   // FPGA-only digital CAN output
    output wire can_l_gpio    // FPGA-only digital CAN output (complement)
);

    wire [7:0]  rx_data;
    wire        rx_valid;
    wire        fifo_space_ok_16;
    
    wire [7:0]  fifo_rdata;
    wire        fifo_rreq;
    wire [6:0]  fifo_count;
    wire        fifo_empty;
    wire        fifo_full;
    
    wire        can_tx;

    assign can_h_gpio = can_tx;
    assign can_l_gpio = ~can_tx;

    // ==================================================================
    // SPI MASTER
    // ==================================================================
    spi_master u_spi_master (
        .clk50            (clk50),
        .rst_n            (rst_n),
        .en               (spi_en),
        .miso             (miso),
        .fifo_space_ok_16 (fifo_space_ok_16),
        .mosi             (mosi),
        .sclk             (sclk),
        .cs_n             (cs_n),
        .rx_data          (rx_data),
        .rx_valid         (rx_valid)
    );

    // ==================================================================
    // FIFO 8x64
    // ==================================================================
    fifo_8x64 u_fifo (
        .clk              (clk50),
        .rst_n            (rst_n),
        .wr_en            (rx_valid),
        .wdata            (rx_data),
        .rd_en            (fifo_rreq),
        .rdata            (fifo_rdata),
        .count            (fifo_count),
        .empty            (fifo_empty),
        .full             (fifo_full),
        .space_ok_16      (fifo_space_ok_16)
    );

    // ==================================================================
    // CAN CONTROLLER
    // ==================================================================
    can_controller #(
        .CAN_ID (11'h123)
    ) u_can_controller (
        .clk50            (clk50),
        .rst_n            (rst_n),
        .fifo_count       (fifo_count),
        .fifo_rdata       (fifo_rdata),
        .fifo_rreq        (fifo_rreq),
        .can_tx           (can_tx)
    );

endmodule


// ==============================================================================
// SPI MASTER
// ==============================================================================
module spi_master (
    input  wire       clk50,
    input  wire       rst_n,
    input  wire       en,
    input  wire       miso,
    input  wire       fifo_space_ok_16,
    output reg        mosi,
    output reg        sclk,
    output reg        cs_n,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);
    localparam IDLE      = 2'd0;
    localparam START     = 2'd1;
    localparam TXRX      = 2'd2;
    localparam WAIT_END  = 2'd3;

    reg [1:0] state;
    reg [3:0] div_cnt;
    reg [2:0] bit_cnt;
    reg [3:0] byte_cnt;
    reg [7:0] rx_shift;

    always @(posedge clk50 or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            mosi     <= 1'b0;
            sclk     <= 1'b0;
            cs_n     <= 1'b1;
            rx_data  <= 8'd0;
            rx_valid <= 1'b0;
            div_cnt  <= 4'd0;
            bit_cnt  <= 3'd0;
            byte_cnt <= 4'd0;
            rx_shift <= 8'd0;
        end else begin
            rx_valid <= 1'b0;
            
            case (state)
                IDLE: begin
                    cs_n <= 1'b1;
                    sclk <= 1'b0;
                    mosi <= 1'b0;
                    if (en && fifo_space_ok_16) begin
                        cs_n     <= 1'b0;
                        state    <= START;
                        div_cnt  <= 4'd0;
                    end
                end
                
                START: begin
                    if (div_cnt == 4'd4) begin
                        div_cnt  <= 4'd0;
                        state    <= TXRX;
                        bit_cnt  <= 3'd0;
                        byte_cnt <= 4'd0;
                        rx_shift <= 8'd0;
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end
                
                TXRX: begin
                    if (div_cnt == 4'd4) begin
                        sclk     <= 1'b1;
                        rx_shift <= {rx_shift[6:0], miso};
                        if (bit_cnt == 3'd7) begin
                            rx_data  <= {rx_shift[6:0], miso};
                            rx_valid <= 1'b1;
                        end
                        div_cnt <= div_cnt + 1'b1;
                    end 
                    else if (div_cnt == 4'd9) begin
                        sclk    <= 1'b0;
                        mosi    <= 1'b0;
                        div_cnt <= 4'd0;
                        if (bit_cnt == 3'd7) begin
                            bit_cnt <= 3'd0;
                            if (byte_cnt == 4'd15) begin
                                state <= WAIT_END;
                            end else begin
                                byte_cnt <= byte_cnt + 1'b1;
                            end
                        end else begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end
                    end 
                    else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end
                
                WAIT_END: begin
                    if (div_cnt == 4'd4) begin
                        cs_n  <= 1'b1;
                        state <= IDLE;
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule


// ==============================================================================
// FIFO 8x64
// ==============================================================================
module fifo_8x64 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       wr_en,
    input  wire [7:0] wdata,
    input  wire       rd_en,
    output reg  [7:0] rdata,
    output reg  [6:0] count,
    output wire       empty,
    output wire       full,
    output wire       space_ok_16
);
    reg [7:0] mem [0:63];
    reg [5:0] wr_ptr;
    reg [5:0] rd_ptr;

    assign empty = (count == 7'd0);
    assign full  = (count == 7'd64);
    assign space_ok_16 = ((7'd64 - count) >= 7'd16);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 6'd0;
            rd_ptr <= 6'd0;
            count  <= 7'd0;
            rdata  <= 8'd0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr] <= wdata;
                wr_ptr      <= wr_ptr + 1'b1;
            end
            if (rd_en && !empty) begin
                rdata  <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1'b1;
            end
            
            case ({wr_en && !full, rd_en && !empty})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: ; // Hold
            endcase
        end
    end
endmodule


// =============================================================================
// CAN CONTROLLER (M_POP_WAIT and 1-bit TX_ACK fix)
// =============================================================================
module can_controller #(
    parameter CAN_ID = 11'h123
)(
    input  wire       clk50,
    input  wire       rst_n,
    input  wire [6:0] fifo_count,
    input  wire [7:0] fifo_rdata,
    output reg        fifo_rreq,
    output reg        can_tx
);

    localparam [2:0] M_IDLE     = 3'd0;
    localparam [2:0] M_POP_REQ  = 3'd1;
    localparam [2:0] M_POP_WAIT = 3'd2;
    localparam [2:0] M_POP_CAP  = 3'd3;
    localparam [2:0] M_TX       = 3'd4;

    localparam [3:0] TX_SOF     = 4'd0;
    localparam [3:0] TX_ID      = 4'd1;
    localparam [3:0] TX_CTRL    = 4'd2;
    localparam [3:0] TX_DLC     = 4'd3;
    localparam [3:0] TX_DATA    = 4'd4;
    localparam [3:0] TX_CRC     = 4'd5;
    localparam [3:0] TX_CRCDEL  = 4'd6;
    localparam [3:0] TX_ACK     = 4'd7;
    localparam [3:0] TX_EOF     = 4'd8;
    localparam [3:0] TX_INTER   = 4'd9;

    reg [2:0] main_st;
    reg [3:0] tx_st;

    reg [6:0] baud_cnt;
    wire      can_tick = (baud_cnt == 7'd99);

    reg [6:0] bit_cnt;
    reg [63:0] data_buf;
    reg [14:0] crc_reg;

    reg [2:0] stuff_cnt;
    reg frame_cnt;
    reg [3:0] pop_cnt;
    reg bit_val;

    always @(posedge clk50 or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt <= 7'd0;
        end else if (can_tick || main_st != M_TX) begin
            baud_cnt <= 7'd0;
        end else begin
            baud_cnt <= baud_cnt + 1'b1;
        end
    end

    function [14:0] next_crc;
        input [14:0] crc_in;
        input        bit_in;
        reg          d;
        begin
            d = crc_in[14] ^ bit_in;
            next_crc = {crc_in[13:0], 1'b0} ^
                       (d ? 15'h4599 : 15'h0000);
        end
    endfunction

    always @(*) begin
        case (tx_st)
            TX_SOF:      bit_val = 1'b0;
            TX_ID:       bit_val = CAN_ID[10 - bit_cnt];
            TX_CTRL:     bit_val = 1'b0;
            TX_DLC:      bit_val = (bit_cnt == 0) ? 1'b1 : 1'b0;
            TX_DATA:     bit_val = data_buf[63 - bit_cnt];
            TX_CRC:      bit_val = crc_reg[14 - bit_cnt];
            TX_CRCDEL:   bit_val = 1'b1;
            TX_ACK:      bit_val = 1'b1;
            TX_EOF:      bit_val = 1'b1;
            TX_INTER:    bit_val = 1'b1;
            default:     bit_val = 1'b1;
        endcase
    end

    wire stuff_enable  = (tx_st <= TX_CRC);
    wire stuff_trigger = (stuff_cnt == 3'd5) && stuff_enable;

    always @(posedge clk50 or negedge rst_n) begin
        if (!rst_n) begin
            main_st   <= M_IDLE;
            tx_st     <= TX_SOF;
            fifo_rreq <= 1'b0;
            can_tx    <= 1'b1;
            bit_cnt   <= 7'd0;
            data_buf  <= 64'd0;
            crc_reg   <= 15'd0;
            stuff_cnt <= 3'd1;
            frame_cnt <= 1'b0;
            pop_cnt   <= 4'd0;
        end else begin
            case (main_st)
                M_IDLE: begin
                    fifo_rreq <= 1'b0;
                    can_tx    <= 1'b1;

                    if (frame_cnt == 1'b0) begin
                        if (fifo_count >= 7'd16) begin
                            pop_cnt <= 4'd0;
                            main_st <= M_POP_REQ;
                        end
                    end
                    else begin
                        if (fifo_count >= 7'd8) begin
                            pop_cnt <= 4'd0;
                            main_st <= M_POP_REQ;
                        end
                    end
                end

                M_POP_REQ: begin
                    fifo_rreq <= 1'b1;
                    main_st   <= M_POP_WAIT;
                end

                M_POP_WAIT: begin
                    fifo_rreq <= 1'b0;
                    main_st   <= M_POP_CAP;
                end

                M_POP_CAP: begin
                    fifo_rreq <= 1'b0;
                    data_buf <= {data_buf[55:0], fifo_rdata};

                    if (pop_cnt == 4'd7) begin
                        main_st   <= M_TX;
                        tx_st     <= TX_SOF;
                        bit_cnt   <= 7'd0;
                        stuff_cnt <= 3'd1;
                        crc_reg   <= 15'd0;
                        can_tx    <= 1'b1;
                        pop_cnt   <= 4'd0;
                    end else begin
                        pop_cnt <= pop_cnt + 1'b1;
                        main_st <= M_POP_REQ;
                    end
                end

                M_TX: begin
                    if (can_tick) begin
                        if (stuff_trigger) begin
                            can_tx    <= ~can_tx;
                            stuff_cnt <= 3'd1;
                        end else begin
                            can_tx <= bit_val;

                            if (stuff_enable) begin
                                if (bit_val == can_tx)
                                    stuff_cnt <= stuff_cnt + 1'b1;
                                else
                                    stuff_cnt <= 3'd1;
                            end else begin
                                stuff_cnt <= 3'd1;
                            end

                            if (tx_st <= TX_DATA)
                                crc_reg <= next_crc(crc_reg, bit_val);

                            case (tx_st)
                                TX_SOF: begin
                                    tx_st   <= TX_ID;
                                    bit_cnt <= 7'd0;
                                end
                                TX_ID: begin
                                    if (bit_cnt == 7'd10) begin
                                        tx_st   <= TX_CTRL;
                                        bit_cnt <= 7'd0;
                                    end else begin
                                        bit_cnt <= bit_cnt + 1'b1;
                                    end
                                end
                                TX_CTRL: begin
                                    if (bit_cnt == 7'd2) begin
                                        tx_st   <= TX_DLC;
                                        bit_cnt <= 7'd0;
                                    end else begin
                                        bit_cnt <= bit_cnt + 1'b1;
                                    end
                                end
                                TX_DLC: begin
                                    if (bit_cnt == 7'd3) begin
                                        tx_st   <= TX_DATA;
                                        bit_cnt <= 7'd0;
                                    end else begin
                                        bit_cnt <= bit_cnt + 1'b1;
                                    end
                                end
                                TX_DATA: begin
                                    if (bit_cnt == 7'd63) begin
                                        tx_st   <= TX_CRC;
                                        bit_cnt <= 7'd0;
                                    end else begin
                                        bit_cnt <= bit_cnt + 1'b1;
                                    end
                                end
                                TX_CRC: begin
                                    if (bit_cnt == 7'd14) begin
                                        tx_st   <= TX_CRCDEL;
                                        bit_cnt <= 7'd0;
                                    end else begin
                                        bit_cnt <= bit_cnt + 1'b1;
                                    end
                                end
                                TX_CRCDEL: begin
                                    tx_st   <= TX_ACK;
                                    bit_cnt <= 7'd0;
                                end
                                TX_ACK: begin
                                    tx_st   <= TX_EOF;
                                    bit_cnt <= 7'd0;
                                end
                                TX_EOF: begin
                                    if (bit_cnt == 7'd6) begin
                                        tx_st   <= TX_INTER;
                                        bit_cnt <= 7'd0;
                                    end else begin
                                        bit_cnt <= bit_cnt + 1'b1;
                                    end
                                end
                                TX_INTER: begin
                                    if (bit_cnt == 7'd2) begin
                                        if (frame_cnt == 1'b0) begin
                                            frame_cnt <= 1'b1;
                                            main_st   <= M_IDLE;
                                        end else begin
                                            frame_cnt <= 1'b0;
                                            main_st   <= M_IDLE;
                                        end
                                    end else begin
                                        bit_cnt <= bit_cnt + 1'b1;
                                    end
                                end
                            endcase
                        end
                    end
                end
                default: begin
                    main_st <= M_IDLE;
                end
            endcase
        end
    end
endmodule