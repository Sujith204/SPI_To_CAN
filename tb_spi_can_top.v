`timescale 1ns/1ps

module tb_spi_can_top;

    reg clk50;
    reg rst_n;
    reg spi_en;
    reg miso;

    wire mosi;
    wire sclk;
    wire cs_n;
    wire can_h_gpio;
    wire can_l_gpio;

    reg [127:0] expected_payload;
    integer spi_bit_idx;

    spi_can_top dut (
        .clk50      (clk50),
        .rst_n      (rst_n),
        .spi_en     (spi_en),
        .miso       (miso),
        .mosi       (mosi),
        .sclk       (sclk),
        .cs_n       (cs_n),
        .can_h_gpio (can_h_gpio),
        .can_l_gpio (can_l_gpio)
    );

    initial begin
        $timeformat(-9, 0, " ns", 0);
    end

    // 50 MHz clock
    initial begin
        clk50 = 1'b0;
        forever #10 clk50 = ~clk50;
    end

    // 16-byte SPI payload
    initial begin
        expected_payload = 128'h112233445566778899AABBCCDDEEFFF1;
    end

    // SPI MODE 0 slave model
    initial begin
        spi_bit_idx = 127;
        miso        = 1'b1;
    end

    always @(negedge cs_n) begin
        spi_bit_idx = 127;
        miso        = expected_payload[127];
    end

    always @(negedge sclk) begin
        if (!cs_n) begin
            if (spi_bit_idx > 0) begin
                spi_bit_idx = spi_bit_idx - 1;
                miso        = expected_payload[spi_bit_idx];
            end
        end
    end

    // ==================================================================
    // FIXED CAN PROTOCOL DECODER TASK (Strict 12-bit trailing consume)
    // ==================================================================
    task decode_can_frame;
        output [10:0] id_out;
        output [3:0]  dlc_out;
        output [63:0] data_out;
        output [14:0] crc_out;

        reg [97:0] raw_frame;
        integer i;
        integer j;

        reg current_bit;
        reg last_bit;
        integer same_bit_count;

        begin
            // Wait for CAN SOF
            @(negedge can_h_gpio);

            // Sample center of SOF
            #1000;

            current_bit = can_h_gpio;
            raw_frame[97] = current_bit;
            last_bit       = current_bit;
            same_bit_count = 1;
            i = 96;

            // Capture 97 remaining semantic bits
            while (i >= 0) begin
                #2000;
                current_bit = can_h_gpio;

                if (same_bit_count == 5) begin
                    same_bit_count = 0;
                    // Stuff bit is discarded. last_bit remains unchanged.
                end else begin
                    raw_frame[i] = current_bit;
                    i = i - 1;

                    if (same_bit_count == 0) begin
                        last_bit       = current_bit;
                        same_bit_count = 1;
                    end else if (current_bit == last_bit) begin
                        same_bit_count = same_bit_count + 1;
                    end else begin
                        last_bit       = current_bit;
                        same_bit_count = 1;
                    end
                end
            end

            // Consume the remaining 12 fixed CAN frame fields
            // CRC delimiter (1) + ACK (1) + EOF (7) + Intermission (3)
            for (j = 0; j < 12; j = j + 1) begin
                #2000;
            end

            // Extract fields
            id_out   = raw_frame[96:86];
            dlc_out  = raw_frame[82:79];
            data_out = raw_frame[78:15];
            crc_out  = raw_frame[14:0];
        end
    endtask

    // Variables for capturing CAN output
    reg [10:0] can1_id, can2_id;
    reg [3:0]  can1_dlc, can2_dlc;
    reg [63:0] can1_data, can2_data;
    reg [14:0] can1_crc, can2_crc;

    // ==================================================================
    // MAIN EXECUTION THREAD
    // ==================================================================
    initial begin
        rst_n  = 1'b0;
        spi_en = 1'b0;

        #200;
        rst_n  = 1'b1;
        #200;

        spi_en = 1'b1;
        @(negedge cs_n);
        spi_en = 1'b0;

        decode_can_frame(can1_id, can1_dlc, can1_data, can1_crc);
        decode_can_frame(can2_id, can2_dlc, can2_data, can2_crc);

        $display("==============================================");
        if ((can1_id   === 11'h123) &&
            (can1_dlc  === 4'h8)    &&
            (can1_data === 64'h1122334455667788) &&
            (can1_crc  === 15'h4237) &&
            (can2_id   === 11'h123) &&
            (can2_dlc  === 4'h8)    &&
            (can2_data === 64'h99AABBCCDDEEFFF1) &&
            (can2_crc  === 15'h5B07)) begin

            $display("VERDICT: PASS!");

        end else begin

            $display("VERDICT: FAIL!");

            $display("CAN1_DATA = %016x", can1_data);
            $display("CAN2_DATA = %016x", can2_data);

            $display("CAN1_ID   = %0x", can1_id);
            $display("CAN2_ID   = %0x", can2_id);

            $display("CAN1_DLC  = %0x", can1_dlc);
            $display("CAN2_DLC  = %0x", can2_dlc);

            $display("CAN1_CRC  = %0x", can1_crc);
            $display("CAN2_CRC  = %0x", can2_crc);

        end
        $display("==============================================");

        #1000;
        $finish;
    end

endmodule