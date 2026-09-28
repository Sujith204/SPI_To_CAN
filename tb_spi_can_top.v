`timescale 1ns/1ps

module tb_spi_can_top;

    // ============================================================
    // DUT signals
    // ============================================================

    reg clk50;
    reg rst_n;
    reg spi_en;

    reg miso;

    wire mosi;
    wire sclk;
    wire cs_n;

    wire can_h_gpio;
    wire can_l_gpio;


    // ============================================================
    // DUT
    // ============================================================

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


    // ============================================================
    // 50 MHz clock
    // 20 ns period
    // ============================================================

    initial begin

        clk50 = 1'b0;

        forever
            #10 clk50 = ~clk50;

    end


    // ============================================================
    // SPI packet
    // ============================================================

    reg [7:0] spi_data [0:15];

    integer i;


    // ============================================================
    // SPI SLAVE MODEL
    //
    // SPI Mode 0
    // FPGA samples MISO on rising SCLK edge.
    // ============================================================

    task send_spi_byte;

        input [7:0] data;

        integer j;

        begin

            for (j = 7; j >= 0; j = j - 1) begin

                // Put MISO data while clock is LOW
                @(negedge sclk);

                miso = data[j];

                // FPGA samples on rising edge
                @(posedge sclk);

            end

        end

    endtask


    // ============================================================
    // MAIN TEST
    // ============================================================

    initial begin

        // --------------------------------------------------------
        // Initial values
        // --------------------------------------------------------

        rst_n  = 1'b0;
        spi_en = 1'b0;
        miso   = 1'b0;


        // --------------------------------------------------------
        // 16-byte SPI packet
        // --------------------------------------------------------

        spi_data[0]  = 8'h11;
        spi_data[1]  = 8'h22;
        spi_data[2]  = 8'h33;
        spi_data[3]  = 8'h44;

        spi_data[4]  = 8'h55;
        spi_data[5]  = 8'h66;
        spi_data[6]  = 8'h77;
        spi_data[7]  = 8'h88;

        spi_data[8]  = 8'h99;
        spi_data[9]  = 8'hAA;
        spi_data[10] = 8'hBB;
        spi_data[11] = 8'hCC;

        spi_data[12] = 8'hDD;
        spi_data[13] = 8'hEE;
        spi_data[14] = 8'hFF;
        spi_data[15] = 8'hF1;


        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------

        #200;

        rst_n = 1'b1;

        #200;


        // --------------------------------------------------------
        // Enable SPI
        // --------------------------------------------------------

        spi_en = 1'b1;


        // --------------------------------------------------------
        // Wait for SPI transaction
        // --------------------------------------------------------

        @(negedge cs_n);


        $display("");
        $display("================================================");
        $display(" SPI TRANSACTION START");
        $display("================================================");
        $display("");


        // --------------------------------------------------------
        // Send 16 bytes
        // --------------------------------------------------------

        for (i = 0; i < 16; i = i + 1) begin

            send_spi_byte(spi_data[i]);

        end


        // --------------------------------------------------------
        // Wait for CS to return HIGH
        // --------------------------------------------------------

        @(posedge cs_n);


        $display("");
        $display("================================================");
        $display(" SPI TRANSACTION COMPLETE");
        $display("================================================");
        $display("");


        // --------------------------------------------------------
        // Allow FIFO and CAN controller to operate
        // --------------------------------------------------------

        #1000000;


        $display("");
        $display("================================================");
        $display(" SIMULATION COMPLETE");
        $display("================================================");
        $display("");


        $finish;

    end


    // ============================================================
    // SPI RECEIVE MONITOR
    // ============================================================

    always @(posedge clk50) begin

        if (dut.rx_valid) begin

            $display(
                "[%0t ns] SPI RX BYTE = %02h | FIFO COUNT = %0d",
                $time,
                dut.rx_data,
                dut.fifo_count
            );

        end

    end


    // ============================================================
    // FIFO WRITE MONITOR
    // ============================================================

    always @(posedge clk50) begin

        if (dut.rx_valid) begin

            $display(
                "[%0t ns] FIFO WRITE = %02h",
                $time,
                dut.rx_data
            );

        end

    end


    // ============================================================
    // FIFO READ MONITOR
    // ============================================================

    always @(posedge clk50) begin

        if (dut.fifo_rreq) begin

            $display(
                "[%0t ns] FIFO READ REQUEST | DATA = %02h | COUNT = %0d",
                $time,
                dut.fifo_rdata,
                dut.fifo_count
            );

        end

    end

endmodule
