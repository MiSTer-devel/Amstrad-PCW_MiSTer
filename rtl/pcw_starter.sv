module pcw_starter(
    input wire clk,
    input wire reset,
    input wire sdram_clk_ref,
    input wire sdram_ready,
    input wire model,
    output logic wr,
    output logic rd,
    output logic [15:0] addr,
    output logic [7:0] data,
    output logic [15:0] exec_addr,
    output logic exec_enable,
    output logic active
);

    localparam BOOT_ROM_END = 16'd275;	// Length of boot rom
    // Rom containing boot rom code to transfer to address 0
    boot_loader boot_loader(
        .address(loader_addr),
        .data(loader_data),
        .model(model)
    );

    reg [15:0] loader_addr;
    reg [7:0] loader_data;
    logic enabled;
    logic reset_ne;

    assign active = enabled;

    edge_det reset_edge_det(
        .clk_sys(clk),
        .signal(reset),
        .neg_edge(reset_ne));

    logic [15:0] read_addr;
    logic [7:0] read_data;
    logic [2:0] state;

    reg sdram_clk_ref_last;

    always @(posedge clk) begin
        if(reset_ne) begin
            loader_addr <= 'b0;
            addr <= 'b0;
            wr <= 1'b0;
            rd <= 1'b0;
            enabled <= 1'b1;
            state <= 2'b00;
            exec_addr <= 'b0;
            exec_enable <= 1'b0;
        end
        else begin
            sdram_clk_ref_last <= sdram_clk_ref;
            if (sdram_ready & ~sdram_clk_ref_last & sdram_clk_ref) begin
                if (enabled) begin
                    case (state)
                        2'b00: begin
                            data <= loader_data;
                            wr <= 1'b1;
                            state <= 2'b01;
                        end
                        2'b01: begin
                            addr <= addr + 1'd1;
                            loader_addr <= loader_addr + 1'd1;
                            wr <= 1'b0;
                            state <= 2'b00;
                            if (loader_addr >= BOOT_ROM_END) begin
                                loader_addr <= 'b0;
                                state <= 2'b10;
                                exec_enable <= 1'b1;
                            end
                        end
                        2'b10: begin
                            enabled <= 1'b0;
                            exec_enable <= 1'b0;
                        end
                    endcase
                end     
            end
        end
    end






endmodule
