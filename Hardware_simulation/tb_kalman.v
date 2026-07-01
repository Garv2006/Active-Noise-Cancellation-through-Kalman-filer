`timescale 1ns / 1ps

module tb_kalman;

    // Parameters
    parameter D_WIDTH = 32;
    parameter N_TAPS  = 32;

    // Inputs
    reg clk;
    reg rst_n;
    reg start;
    reg signed [D_WIDTH-1:0] d_in;
    reg signed [D_WIDTH-1:0] u_in;

    // Outputs
    wire signed [D_WIDTH-1:0] y_out;
    wire valid;
    wire done;

    // Instantiate the Unit Under Test (UUT)
    kalman_anc_core #(
        .D_WIDTH(D_WIDTH),
        .N_TAPS(N_TAPS)
    ) uut (
        .clk(clk), 
        .rst_n(rst_n), 
        .start(start), 
        .d_in(d_in), 
        .u_in(u_in), 
        .y_out(y_out), 
        .valid(valid), 
        .done(done)
    );

    // Clock generation (100 MHz)
    always #5 clk = ~clk;

    initial begin
        // 1. Tell Icarus to dump signals for GTKWave
        $dumpfile("kalman_waves.vcd");
        $dumpvars(0, tb_kalman); // Dump all variables in this module and below

        // 2. Initialize Inputs
        clk = 0;
        rst_n = 0;
        start = 0;
        d_in = 0;
        u_in = 0;

        // 3. Release Reset
        #20;
        rst_n = 1;
        #20;

        // 4. Simulate a few sample cycles
        // Sample 1
        @(posedge clk);
        start = 1;
        d_in = 32'h0000_1000; // Dummy fixed-point data
        u_in = 32'h0000_0800;
        @(posedge clk);
        start = 0;
        
        // Wait for the FSM to finish processing the sample
        wait(done == 1);
        
        // Delay before next sample (simulating sample rate)
        #100; 

        // Sample 2
        @(posedge clk);
        start = 1;
        d_in = -32'h0000_0500;
        u_in = 32'h0000_0200;
        @(posedge clk);
        start = 0;
        
        wait(done == 1);
        
        #200;
        
        // 5. End simulation
        $display("Simulation Complete.");
        $finish;
    end
endmodule