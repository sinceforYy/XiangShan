/* verilator lint_off UNOPTFLAT */

module EICG_wrapper(
    input CK,
    input TE,
    input E,
    output Q
);
    reg en_latched /*verilator clock_enable*/;

    always @(*) begin
        if (!CK) begin
            en_latched = E || TE;
        end
    end

    assign Q = en_latched && CK;

endmodule