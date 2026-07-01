`timescale 1ns / 1ps

// ==============================================================================
// MAC Unit (Multiply-Accumulate) for Fixed-Point Q15.16
// ==============================================================================
module mac_unit #(
    parameter D_WIDTH = 32,
    parameter Q_FRAC = 16
)(
    input wire clk,
    input wire rst_n,
    input wire clear,
    input wire en,
    input wire signed [D_WIDTH-1:0] a,
    input wire signed [D_WIDTH-1:0] b,
    output reg signed [D_WIDTH-1:0] accum
);
    wire signed [2*D_WIDTH-1:0] mult_result;
    assign mult_result = (a * b) >>> Q_FRAC;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accum <= 0;
        end else if (clear) begin
            accum <= 0;
        end else if (en) begin
            accum <= accum + mult_result[D_WIDTH-1:0]; 
        end
    end
endmodule

// ==============================================================================
// Behavioral Divider (Placeholder for standard IP Core)
// ==============================================================================
module behavioral_divider #(
    parameter D_WIDTH = 32,
    parameter Q_FRAC = 16
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire signed [D_WIDTH-1:0] num,
    input wire signed [D_WIDTH-1:0] den,
    output reg signed [D_WIDTH-1:0] quotient,
    output reg done
);
    // Note: In a real FPGA, replace this with a DSP Divider Generator IP
    // taking multiple clock cycles. This behavioral model takes 1 cycle.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quotient <= 0;
            done <= 0;
        end else if (start) begin
            if (den != 0) begin
                // Shift numerator for fixed-point division
                quotient <= (num <<< Q_FRAC) / den; 
            end else begin
                quotient <= 0; // Avoid divide by zero
            end
            done <= 1;
        end else begin
            done <= 0;
        end
    end
endmodule

// ==============================================================================
// Main Kalman Filter ANC Core
// ==============================================================================
module kalman_anc_core #(
    parameter D_WIDTH = 32,
    parameter Q_FRAC = 16,
    parameter N_TAPS  = 32
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire signed [D_WIDTH-1:0] d_in, // Primary noise
    input wire signed [D_WIDTH-1:0] u_in, // Reference signal
    
    output reg signed [D_WIDTH-1:0] y_out, // Anti-noise
    output reg valid,
    output reg done
);

    // --------------------------------------------------------
    // Memories (Inferred BRAMs) and Registers
    // --------------------------------------------------------
    reg signed [D_WIDTH-1:0] u_buffer [0:N_TAPS-1];       // Reference delay line C(n)
    reg signed [D_WIDTH-1:0] x_state [0:N_TAPS-1];        // State vector x(n)
    reg signed [D_WIDTH-1:0] K_cov [0:N_TAPS-1][0:N_TAPS-1]; // Covariance Matrix K(n)
    
    reg signed [D_WIDTH-1:0] a_priori_err;
    reg signed [D_WIDTH-1:0] kalman_gain [0:N_TAPS-1];

    // Constant Noise Parameters (Adjust based on Q-format)
    localparam signed [D_WIDTH-1:0] Q1 = 32'h0000_0010; // State Noise 
    localparam signed [D_WIDTH-1:0] Q2 = 32'h0000_1999; // Measurement Noise (~0.1)

    // --------------------------------------------------------
    // FSM States
    // --------------------------------------------------------
    localparam IDLE         = 3'd0;
    localparam SHIFT_U      = 3'd1;
    localparam FILTER_OUT   = 3'd2;
    localparam DENOM_CALC   = 3'd3;
    localparam DIVIDE       = 3'd4;
    localparam GAIN_UPDATE  = 3'd5;
    localparam STATE_UPDATE = 3'd6;
    localparam COV_UPDATE   = 3'd7;

    reg [2:0] current_state, next_state;
    reg [5:0] row_cnt, col_cnt;

    // --------------------------------------------------------
    // Datapath Control Signals
    // --------------------------------------------------------
    reg mac_clear, mac_en;
    reg signed [D_WIDTH-1:0] mac_a, mac_b;
    wire signed [D_WIDTH-1:0] mac_accum;
    
    reg div_start;
    wire signed [D_WIDTH-1:0] div_quotient;
    wire div_done;

    // Instantiations
    mac_unit #(.D_WIDTH(D_WIDTH), .Q_FRAC(Q_FRAC)) MAC (
        .clk(clk), .rst_n(rst_n), .clear(mac_clear), .en(mac_en),
        .a(mac_a), .b(mac_b), .accum(mac_accum)
    );

    behavioral_divider #(.D_WIDTH(D_WIDTH), .Q_FRAC(Q_FRAC)) DIV (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .num(32'h0001_0000), .den(mac_accum + Q2), // 1.0 / (denom + Q2)
        .quotient(div_quotient), .done(div_done)
    );

    // --------------------------------------------------------
    // State Transition Logic
    // --------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) current_state <= IDLE;
        else current_state <= next_state;
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE:         if (start) next_state = SHIFT_U;
            SHIFT_U:      next_state = FILTER_OUT;
            FILTER_OUT:   if (row_cnt == N_TAPS - 1) next_state = DENOM_CALC;
            DENOM_CALC:   if (row_cnt == N_TAPS - 1 && col_cnt == N_TAPS - 1) next_state = DIVIDE;
            DIVIDE:       if (div_done) next_state = GAIN_UPDATE;
            GAIN_UPDATE:  if (row_cnt == N_TAPS - 1) next_state = STATE_UPDATE;
            STATE_UPDATE: if (row_cnt == N_TAPS - 1) next_state = COV_UPDATE;
            COV_UPDATE:   if (row_cnt == N_TAPS - 1 && col_cnt == N_TAPS - 1) next_state = IDLE;
        endcase
    end

    // --------------------------------------------------------
    // Datapath Execution
    // --------------------------------------------------------
    integer i, j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_cnt <= 0; col_cnt <= 0;
            valid <= 0; done <= 0; y_out <= 0;
            mac_clear <= 1; mac_en <= 0; div_start <= 0;
            
            for (i = 0; i < N_TAPS; i = i + 1) begin
                u_buffer[i] <= 0;
                x_state[i] <= 0;
                kalman_gain[i] <= 0;
                for (j = 0; j < N_TAPS; j = j + 1) begin
                    K_cov[i][j] <= (i == j) ? 32'h0001_0000 : 0; // Init to Identity
                end
            end
        end else begin
            // Default signal states
            mac_en <= 0; 
            div_start <= 0;
            done <= 0;
            valid <= 0;

            case (current_state)
                IDLE: begin
                    row_cnt <= 0; col_cnt <= 0;
                    mac_clear <= 1;
                end

                SHIFT_U: begin
                    // Shift delay line
                    for (i = N_TAPS-1; i > 0; i = i - 1) u_buffer[i] <= u_buffer[i-1];
                    u_buffer[0] <= u_in;
                    mac_clear <= 1;
                end

                FILTER_OUT: begin
                    mac_clear <= 0;
                    mac_en <= 1;
                    mac_a <= u_buffer[row_cnt];
                    mac_b <= x_state[row_cnt];
                    
                    if (row_cnt < N_TAPS - 1) begin
                        row_cnt <= row_cnt + 1;
                    end else begin
                        row_cnt <= 0;
                        mac_clear <= 1;
                    end
                end
                
                DENOM_CALC: begin
                    // Capture filter output on first cycle of this state
                    if (row_cnt == 0 && col_cnt == 0) begin
                        y_out <= mac_accum;
                        a_priori_err <= d_in - mac_accum;
                        valid <= 1; 
                    end
                    
                    mac_clear <= 0;
                    mac_en <= 1;
                    // C(n) * K * C(n)'
                    mac_a <= u_buffer[row_cnt];
                    mac_b <= K_cov[row_cnt][col_cnt]; 
                    
                    if (col_cnt < N_TAPS - 1) begin
                        col_cnt <= col_cnt + 1;
                    end else begin
                        col_cnt <= 0;
                        if (row_cnt < N_TAPS - 1) row_cnt <= row_cnt + 1;
                        else begin
                            row_cnt <= 0;
                            div_start <= 1; // Trigger division next
                        end
                    end
                end

                DIVIDE: begin
                    mac_clear <= 1;
                    // Waiting for div_done...
                end

                GAIN_UPDATE: begin
                    // Simplifying G computation for structural demonstration
                    // In a true implementation, this uses a MAC for K*C' first, then multiplies by quotient
                    kalman_gain[row_cnt] <= div_quotient; // Placeholder mapping
                    
                    if (row_cnt < N_TAPS - 1) row_cnt <= row_cnt + 1;
                    else row_cnt <= 0;
                end

                STATE_UPDATE: begin
                    mac_clear <= 0;
                    mac_en <= 1;
                    mac_a <= kalman_gain[row_cnt];
                    mac_b <= a_priori_err;
                    
                    // x = x + G * error
                    x_state[row_cnt] <= x_state[row_cnt] + mac_accum; 
                    
                    if (row_cnt < N_TAPS - 1) row_cnt <= row_cnt + 1;
                    else begin
                        row_cnt <= 0;
                        mac_clear <= 1;
                    end
                end

                COV_UPDATE: begin
                    // K = K - G*C*K + Q1
                    // Structural mapping (Simplified for demonstration loop)
                    if (col_cnt < N_TAPS - 1) begin
                        col_cnt <= col_cnt + 1;
                    end else begin
                        col_cnt <= 0;
                        if (row_cnt < N_TAPS - 1) begin
                            row_cnt <= row_cnt + 1;
                        end else begin
                            row_cnt <= 0;
                            done <= 1;
                        end
                    end
                end
            endcase
        end
    end
endmodule