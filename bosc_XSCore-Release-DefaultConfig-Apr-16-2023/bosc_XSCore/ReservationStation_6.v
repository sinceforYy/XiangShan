module ReservationStation_6(
  input          clock,
  input          reset,
  input          io_redirect_valid,
  input          io_redirect_bits_robIdx_flag,
  input  [4:0]   io_redirect_bits_robIdx_value,
  input          io_redirect_bits_level,
  output         io_fromDispatch_0_ready,
  input          io_fromDispatch_0_valid,
  input          io_fromDispatch_0_bits_cf_pd_isRVC,
  input  [1:0]   io_fromDispatch_0_bits_cf_pd_brType,
  input          io_fromDispatch_0_bits_cf_pd_isCall,
  input          io_fromDispatch_0_bits_cf_pd_isRet,
  input          io_fromDispatch_0_bits_cf_pred_taken,
  input          io_fromDispatch_0_bits_cf_ftqPtr_flag,
  input  [2:0]   io_fromDispatch_0_bits_cf_ftqPtr_value,
  input  [2:0]   io_fromDispatch_0_bits_cf_ftqOffset,
  input  [1:0]   io_fromDispatch_0_bits_ctrl_srcType_0,
  input  [1:0]   io_fromDispatch_0_bits_ctrl_srcType_1,
  input  [1:0]   io_fromDispatch_0_bits_ctrl_srcType_2,
  input  [3:0]   io_fromDispatch_0_bits_ctrl_fuType,
  input  [6:0]   io_fromDispatch_0_bits_ctrl_fuOpType,
  input          io_fromDispatch_0_bits_ctrl_rfWen,
  input          io_fromDispatch_0_bits_ctrl_fpWen,
  input  [19:0]  io_fromDispatch_0_bits_ctrl_imm,
  input          io_fromDispatch_0_bits_ctrl_fpu_isAddSub,
  input          io_fromDispatch_0_bits_ctrl_fpu_typeTagIn,
  input          io_fromDispatch_0_bits_ctrl_fpu_typeTagOut,
  input          io_fromDispatch_0_bits_ctrl_fpu_fromInt,
  input          io_fromDispatch_0_bits_ctrl_fpu_wflags,
  input          io_fromDispatch_0_bits_ctrl_fpu_fpWen,
  input  [1:0]   io_fromDispatch_0_bits_ctrl_fpu_fmaCmd,
  input          io_fromDispatch_0_bits_ctrl_fpu_div,
  input          io_fromDispatch_0_bits_ctrl_fpu_sqrt,
  input          io_fromDispatch_0_bits_ctrl_fpu_fcvt,
  input  [1:0]   io_fromDispatch_0_bits_ctrl_fpu_typ,
  input  [1:0]   io_fromDispatch_0_bits_ctrl_fpu_fmt,
  input          io_fromDispatch_0_bits_ctrl_fpu_ren3,
  input  [2:0]   io_fromDispatch_0_bits_ctrl_fpu_rm,
  input          io_fromDispatch_0_bits_srcState_0,
  input          io_fromDispatch_0_bits_srcState_1,
  input          io_fromDispatch_0_bits_srcState_2,
  input  [5:0]   io_fromDispatch_0_bits_psrc_0,
  input  [5:0]   io_fromDispatch_0_bits_psrc_1,
  input  [5:0]   io_fromDispatch_0_bits_psrc_2,
  input  [5:0]   io_fromDispatch_0_bits_pdest,
  input          io_fromDispatch_0_bits_robIdx_flag,
  input  [4:0]   io_fromDispatch_0_bits_robIdx_value,
  input  [63:0]  io_srcRegValue_0_0,
  input  [63:0]  io_srcRegValue_0_1,
  input  [63:0]  io_srcRegValue_0_2,
  input          io_deq_0_ready,
  output         io_deq_0_valid,
  output         io_deq_0_bits_uop_ctrl_rfWen,
  output         io_deq_0_bits_uop_ctrl_fpWen,
  output         io_deq_0_bits_uop_ctrl_fpu_isAddSub,
  output         io_deq_0_bits_uop_ctrl_fpu_typeTagIn,
  output [1:0]   io_deq_0_bits_uop_ctrl_fpu_fmaCmd,
  output         io_deq_0_bits_uop_ctrl_fpu_ren3,
  output [2:0]   io_deq_0_bits_uop_ctrl_fpu_rm,
  output [5:0]   io_deq_0_bits_uop_pdest,
  output         io_deq_0_bits_uop_robIdx_flag,
  output [4:0]   io_deq_0_bits_uop_robIdx_value,
  output [63:0]  io_deq_0_bits_src_0,
  output [63:0]  io_deq_0_bits_src_1,
  output [63:0]  io_deq_0_bits_src_2,
  input          io_fastUopsIn_0_valid,
  input          io_fastUopsIn_0_bits_ctrl_fpWen,
  input  [5:0]   io_fastUopsIn_0_bits_pdest,
  input  [63:0]  io_fastDatas_0,
  input          io_slowPorts_0_valid,
  input          io_slowPorts_0_bits_uop_ctrl_fpWen,
  input  [5:0]   io_slowPorts_0_bits_uop_pdest,
  input  [63:0]  io_slowPorts_0_bits_data,
  input          io_slowPorts_1_valid,
  input          io_slowPorts_1_bits_uop_ctrl_fpWen,
  input  [5:0]   io_slowPorts_1_bits_uop_pdest,
  input  [63:0]  io_slowPorts_1_bits_data,
  input          io_slowPorts_2_valid,
  input          io_slowPorts_2_bits_uop_ctrl_fpWen,
  input  [5:0]   io_slowPorts_2_bits_uop_pdest,
  input  [63:0]  io_slowPorts_2_bits_data,
  input          io_slowPorts_3_valid,
  input          io_slowPorts_3_bits_uop_ctrl_fpWen,
  input  [5:0]   io_slowPorts_3_bits_uop_pdest,
  input  [63:0]  io_slowPorts_3_bits_data,
  output         io_fmaMid_0_in_valid,
  output         io_fmaMid_0_in_bits_fp_prod_sign,
  output [10:0]  io_fmaMid_0_in_bits_fp_prod_exp,
  output [104:0] io_fmaMid_0_in_bits_fp_prod_sig,
  output         io_fmaMid_0_in_bits_inter_flags_isNaN,
  output         io_fmaMid_0_in_bits_inter_flags_isInf,
  output         io_fmaMid_0_in_bits_inter_flags_isInv,
  output         io_fmaMid_0_in_bits_inter_flags_overflow,
  input          io_fmaMid_0_out_bits_fp_prod_sign,
  input  [10:0]  io_fmaMid_0_out_bits_fp_prod_exp,
  input  [104:0] io_fmaMid_0_out_bits_fp_prod_sig,
  input          io_fmaMid_0_out_bits_inter_flags_isNaN,
  input          io_fmaMid_0_out_bits_inter_flags_isInf,
  input          io_fmaMid_0_out_bits_inter_flags_isInv,
  output         io_fmaMid_0_waitForAdd,
  output [5:0]   io_perf_0_value
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [31:0] _RAND_6;
  reg [31:0] _RAND_7;
  reg [31:0] _RAND_8;
  reg [31:0] _RAND_9;
  reg [31:0] _RAND_10;
  reg [31:0] _RAND_11;
  reg [31:0] _RAND_12;
  reg [31:0] _RAND_13;
  reg [31:0] _RAND_14;
  reg [31:0] _RAND_15;
  reg [31:0] _RAND_16;
  reg [31:0] _RAND_17;
  reg [31:0] _RAND_18;
  reg [31:0] _RAND_19;
  reg [31:0] _RAND_20;
  reg [31:0] _RAND_21;
  reg [31:0] _RAND_22;
  reg [31:0] _RAND_23;
  reg [31:0] _RAND_24;
  reg [31:0] _RAND_25;
  reg [31:0] _RAND_26;
  reg [31:0] _RAND_27;
  reg [31:0] _RAND_28;
  reg [31:0] _RAND_29;
  reg [31:0] _RAND_30;
  reg [31:0] _RAND_31;
  reg [31:0] _RAND_32;
  reg [31:0] _RAND_33;
  reg [31:0] _RAND_34;
  reg [31:0] _RAND_35;
  reg [31:0] _RAND_36;
  reg [31:0] _RAND_37;
  reg [31:0] _RAND_38;
  reg [31:0] _RAND_39;
  reg [31:0] _RAND_40;
  reg [31:0] _RAND_41;
  reg [31:0] _RAND_42;
  reg [31:0] _RAND_43;
  reg [31:0] _RAND_44;
  reg [31:0] _RAND_45;
  reg [31:0] _RAND_46;
  reg [31:0] _RAND_47;
  reg [31:0] _RAND_48;
  reg [31:0] _RAND_49;
  reg [31:0] _RAND_50;
  reg [31:0] _RAND_51;
  reg [31:0] _RAND_52;
  reg [31:0] _RAND_53;
  reg [31:0] _RAND_54;
  reg [31:0] _RAND_55;
  reg [31:0] _RAND_56;
  reg [31:0] _RAND_57;
  reg [31:0] _RAND_58;
  reg [31:0] _RAND_59;
  reg [31:0] _RAND_60;
  reg [31:0] _RAND_61;
  reg [31:0] _RAND_62;
  reg [31:0] _RAND_63;
  reg [31:0] _RAND_64;
  reg [31:0] _RAND_65;
  reg [31:0] _RAND_66;
  reg [31:0] _RAND_67;
  reg [31:0] _RAND_68;
  reg [31:0] _RAND_69;
  reg [31:0] _RAND_70;
  reg [31:0] _RAND_71;
  reg [31:0] _RAND_72;
  reg [31:0] _RAND_73;
  reg [31:0] _RAND_74;
  reg [31:0] _RAND_75;
  reg [31:0] _RAND_76;
  reg [31:0] _RAND_77;
  reg [31:0] _RAND_78;
  reg [31:0] _RAND_79;
  reg [31:0] _RAND_80;
  reg [31:0] _RAND_81;
  reg [31:0] _RAND_82;
  reg [31:0] _RAND_83;
  reg [31:0] _RAND_84;
  reg [31:0] _RAND_85;
  reg [31:0] _RAND_86;
  reg [31:0] _RAND_87;
  reg [31:0] _RAND_88;
  reg [31:0] _RAND_89;
  reg [31:0] _RAND_90;
  reg [31:0] _RAND_91;
  reg [31:0] _RAND_92;
  reg [31:0] _RAND_93;
  reg [31:0] _RAND_94;
  reg [31:0] _RAND_95;
  reg [31:0] _RAND_96;
  reg [31:0] _RAND_97;
  reg [31:0] _RAND_98;
  reg [31:0] _RAND_99;
  reg [63:0] _RAND_100;
  reg [31:0] _RAND_101;
  reg [63:0] _RAND_102;
  reg [31:0] _RAND_103;
  reg [63:0] _RAND_104;
  reg [31:0] _RAND_105;
  reg [63:0] _RAND_106;
  reg [31:0] _RAND_107;
  reg [31:0] _RAND_108;
  reg [31:0] _RAND_109;
  reg [31:0] _RAND_110;
  reg [31:0] _RAND_111;
  reg [31:0] _RAND_112;
  reg [31:0] _RAND_113;
  reg [31:0] _RAND_114;
  reg [31:0] _RAND_115;
  reg [31:0] _RAND_116;
  reg [31:0] _RAND_117;
  reg [31:0] _RAND_118;
  reg [31:0] _RAND_119;
  reg [31:0] _RAND_120;
  reg [31:0] _RAND_121;
  reg [31:0] _RAND_122;
  reg [31:0] _RAND_123;
  reg [31:0] _RAND_124;
  reg [31:0] _RAND_125;
  reg [31:0] _RAND_126;
  reg [31:0] _RAND_127;
  reg [31:0] _RAND_128;
  reg [31:0] _RAND_129;
  reg [31:0] _RAND_130;
  reg [31:0] _RAND_131;
  reg [31:0] _RAND_132;
  reg [31:0] _RAND_133;
  reg [31:0] _RAND_134;
  reg [31:0] _RAND_135;
  reg [31:0] _RAND_136;
  reg [31:0] _RAND_137;
  reg [31:0] _RAND_138;
  reg [31:0] _RAND_139;
  reg [31:0] _RAND_140;
  reg [31:0] _RAND_141;
  reg [31:0] _RAND_142;
  reg [31:0] _RAND_143;
  reg [31:0] _RAND_144;
  reg [31:0] _RAND_145;
  reg [31:0] _RAND_146;
  reg [31:0] _RAND_147;
  reg [31:0] _RAND_148;
  reg [31:0] _RAND_149;
  reg [31:0] _RAND_150;
  reg [31:0] _RAND_151;
  reg [31:0] _RAND_152;
  reg [31:0] _RAND_153;
  reg [127:0] _RAND_154;
  reg [31:0] _RAND_155;
  reg [31:0] _RAND_156;
  reg [31:0] _RAND_157;
  reg [31:0] _RAND_158;
  reg [31:0] _RAND_159;
`endif // RANDOMIZE_REG_INIT
  wire  statusArray_clock; // @[ReservationStation.scala 261:27]
  wire  statusArray_reset; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_redirect_valid; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_redirect_bits_robIdx_flag; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_redirect_bits_robIdx_value; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_redirect_bits_level; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_isValid; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_isValidNext; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_canIssue; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_flushed; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_enable; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_update_0_addr; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_data_scheduled; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_data_srcState_0; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_data_srcState_1; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_data_srcState_2; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_data_midState; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_update_0_data_psrc_0; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_update_0_data_psrc_1; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_update_0_data_psrc_2; // @[ReservationStation.scala 261:27]
  wire [1:0] statusArray_io_update_0_data_srcType_0; // @[ReservationStation.scala 261:27]
  wire [1:0] statusArray_io_update_0_data_srcType_1; // @[ReservationStation.scala 261:27]
  wire [1:0] statusArray_io_update_0_data_srcType_2; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_data_robIdx_flag; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_update_0_data_robIdx_value; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_update_0_data_isFirstIssue; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_0_valid; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_0_bits_ctrl_fpWen; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_wakeup_0_bits_pdest; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_1_valid; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_1_bits_ctrl_fpWen; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_wakeup_1_bits_pdest; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_2_valid; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_2_bits_ctrl_fpWen; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_wakeup_2_bits_pdest; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_3_valid; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_3_bits_ctrl_fpWen; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_wakeup_3_bits_pdest; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_4_valid; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_wakeup_4_bits_ctrl_fpWen; // @[ReservationStation.scala 261:27]
  wire [5:0] statusArray_io_wakeup_4_bits_pdest; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_0_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_0_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_0_2; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_1_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_1_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_1_2; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_2_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_2_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_2_2; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_3_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_3_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_3_2; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_4_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_4_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_4_2; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_5_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_5_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_5_2; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_6_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_6_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_6_2; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_7_0; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_7_1; // @[ReservationStation.scala 261:27]
  wire [4:0] statusArray_io_wakeupMatch_7_2; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_issueGranted_0_valid; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_issueGranted_0_bits; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_issueGranted_1_valid; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_issueGranted_1_bits; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_issueGranted_2_valid; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_issueGranted_2_bits; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_isFirstIssue_1; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_isFirstIssue_2; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_allSrcReady_1; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_allSrcReady_2; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_updateMidState; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_deqResp_0_valid; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_deqResp_0_bits_rsMask; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_deqResp_0_bits_success; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_deqResp_1_valid; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_deqResp_1_bits_rsMask; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_deqResp_1_bits_success; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_deqResp_2_valid; // @[ReservationStation.scala 261:27]
  wire [7:0] statusArray_io_deqResp_2_bits_rsMask; // @[ReservationStation.scala 261:27]
  wire  statusArray_io_deqResp_2_bits_success; // @[ReservationStation.scala 261:27]
  wire [7:0] select_io_validVec; // @[ReservationStation.scala 262:22]
  wire [7:0] select_io_allocate_0_bits; // @[ReservationStation.scala 262:22]
  wire [7:0] select_io_request; // @[ReservationStation.scala 262:22]
  wire  select_io_grant_0_valid; // @[ReservationStation.scala 262:22]
  wire [7:0] select_io_grant_0_bits; // @[ReservationStation.scala 262:22]
  wire  dataArray_clock; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_read_0_addr; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_read_0_data_0; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_read_0_data_1; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_read_0_data_2; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_read_1_addr; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_read_1_data_0; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_read_1_data_1; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_read_1_data_2; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_write_0_enable; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_write_0_mask_0; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_write_0_mask_1; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_write_0_mask_2; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_write_0_addr; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_write_0_data_0; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_write_0_data_1; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_write_0_data_2; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_multiWrite_0_enable; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_0_addr_0; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_0_addr_1; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_0_addr_2; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_multiWrite_0_data; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_multiWrite_1_enable; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_1_addr_0; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_1_addr_1; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_1_addr_2; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_multiWrite_1_data; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_multiWrite_2_enable; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_2_addr_0; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_2_addr_1; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_2_addr_2; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_multiWrite_2_data; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_multiWrite_3_enable; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_3_addr_0; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_3_addr_1; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_multiWrite_3_addr_2; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_multiWrite_3_data; // @[ReservationStation.scala 263:25]
  wire  dataArray_io_partialWrite_0_enable; // @[ReservationStation.scala 263:25]
  wire [7:0] dataArray_io_partialWrite_0_addr; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_partialWrite_0_data_0; // @[ReservationStation.scala 263:25]
  wire [63:0] dataArray_io_partialWrite_0_data_1; // @[ReservationStation.scala 263:25]
  wire  payloadArray_clock; // @[ReservationStation.scala 264:28]
  wire [7:0] payloadArray_io_read_0_addr; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_cf_pd_isRVC; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_0_data_cf_pd_brType; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_cf_pd_isCall; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_cf_pd_isRet; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_cf_pred_taken; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_cf_ftqPtr_flag; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_read_0_data_cf_ftqPtr_value; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_read_0_data_cf_ftqOffset; // @[ReservationStation.scala 264:28]
  wire [3:0] payloadArray_io_read_0_data_ctrl_fuType; // @[ReservationStation.scala 264:28]
  wire [6:0] payloadArray_io_read_0_data_ctrl_fuOpType; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_rfWen; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpWen; // @[ReservationStation.scala 264:28]
  wire [19:0] payloadArray_io_read_0_data_ctrl_imm; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_isAddSub; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_typeTagOut; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_fromInt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_wflags; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_fpWen; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_0_data_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_div; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_sqrt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_fcvt; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_0_data_ctrl_fpu_typ; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_0_data_ctrl_fpu_fmt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_ctrl_fpu_ren3; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_read_0_data_ctrl_fpu_rm; // @[ReservationStation.scala 264:28]
  wire [5:0] payloadArray_io_read_0_data_pdest; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_0_data_robIdx_flag; // @[ReservationStation.scala 264:28]
  wire [4:0] payloadArray_io_read_0_data_robIdx_value; // @[ReservationStation.scala 264:28]
  wire [7:0] payloadArray_io_read_1_addr; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_cf_pd_isRVC; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_1_data_cf_pd_brType; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_cf_pd_isCall; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_cf_pd_isRet; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_cf_pred_taken; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_cf_ftqPtr_flag; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_read_1_data_cf_ftqPtr_value; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_read_1_data_cf_ftqOffset; // @[ReservationStation.scala 264:28]
  wire [3:0] payloadArray_io_read_1_data_ctrl_fuType; // @[ReservationStation.scala 264:28]
  wire [6:0] payloadArray_io_read_1_data_ctrl_fuOpType; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_rfWen; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpWen; // @[ReservationStation.scala 264:28]
  wire [19:0] payloadArray_io_read_1_data_ctrl_imm; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_isAddSub; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_typeTagOut; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_fromInt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_wflags; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_fpWen; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_1_data_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_div; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_sqrt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_fcvt; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_1_data_ctrl_fpu_typ; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_read_1_data_ctrl_fpu_fmt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_ctrl_fpu_ren3; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_read_1_data_ctrl_fpu_rm; // @[ReservationStation.scala 264:28]
  wire [5:0] payloadArray_io_read_1_data_pdest; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_read_1_data_robIdx_flag; // @[ReservationStation.scala 264:28]
  wire [4:0] payloadArray_io_read_1_data_robIdx_value; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_enable; // @[ReservationStation.scala 264:28]
  wire [7:0] payloadArray_io_write_0_addr; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_cf_pd_isRVC; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_write_0_data_cf_pd_brType; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_cf_pd_isCall; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_cf_pd_isRet; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_cf_pred_taken; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_cf_ftqPtr_flag; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_write_0_data_cf_ftqPtr_value; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_write_0_data_cf_ftqOffset; // @[ReservationStation.scala 264:28]
  wire [3:0] payloadArray_io_write_0_data_ctrl_fuType; // @[ReservationStation.scala 264:28]
  wire [6:0] payloadArray_io_write_0_data_ctrl_fuOpType; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_rfWen; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpWen; // @[ReservationStation.scala 264:28]
  wire [19:0] payloadArray_io_write_0_data_ctrl_imm; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_isAddSub; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_typeTagOut; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_fromInt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_wflags; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_fpWen; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_write_0_data_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_div; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_sqrt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_fcvt; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_write_0_data_ctrl_fpu_typ; // @[ReservationStation.scala 264:28]
  wire [1:0] payloadArray_io_write_0_data_ctrl_fpu_fmt; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_ctrl_fpu_ren3; // @[ReservationStation.scala 264:28]
  wire [2:0] payloadArray_io_write_0_data_ctrl_fpu_rm; // @[ReservationStation.scala 264:28]
  wire [5:0] payloadArray_io_write_0_data_pdest; // @[ReservationStation.scala 264:28]
  wire  payloadArray_io_write_0_data_robIdx_flag; // @[ReservationStation.scala 264:28]
  wire [4:0] payloadArray_io_write_0_data_robIdx_value; // @[ReservationStation.scala 264:28]
  wire  s1_oldestSel_age_clock; // @[SelectPolicy.scala 174:21]
  wire  s1_oldestSel_age_reset; // @[SelectPolicy.scala 174:21]
  wire [7:0] s1_oldestSel_age_io_enq_0; // @[SelectPolicy.scala 174:21]
  wire [7:0] s1_oldestSel_age_io_deq; // @[SelectPolicy.scala 174:21]
  wire [7:0] s1_oldestSel_age_io_out; // @[SelectPolicy.scala 174:21]
  wire  oldestSelection_io_oldest_valid; // @[ReservationStation.scala 499:33]
  wire  oldestSelection_io_isOverrided_0; // @[ReservationStation.scala 499:33]
  wire [63:0] immExt_io_data_in_0; // @[DataArray.scala 161:18]
  wire [63:0] immExt_io_data_in_1; // @[DataArray.scala 161:18]
  wire [63:0] immExt_io_data_in_2; // @[DataArray.scala 161:18]
  wire [63:0] immExt_io_data_out_0; // @[DataArray.scala 161:18]
  wire [63:0] immExt_io_data_out_1; // @[DataArray.scala 161:18]
  wire [63:0] immExt_io_data_out_2; // @[DataArray.scala 161:18]
  wire  dataSelect_io_doOverride_0; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_readData_0_0; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_readData_0_1; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_readData_0_2; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_readData_1_0; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_readData_1_1; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_readData_1_2; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_0_0; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_0_1; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_0_2; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_1_0; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_1_1; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_1_2; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_2_0; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_2_1; // @[ReservationStation.scala 691:26]
  wire [3:0] dataSelect_io_fromSlowPorts_2_2; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_slowData_0; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_slowData_1; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_slowData_2; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_slowData_3; // @[ReservationStation.scala 691:26]
  wire  dataSelect_io_enqBypass_0_0; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_enqData_0_0_bits; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_enqData_0_1_bits; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_enqData_0_2_bits; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_deqData_0_0; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_deqData_0_1; // @[ReservationStation.scala 691:26]
  wire [63:0] dataSelect_io_deqData_0_2; // @[ReservationStation.scala 691:26]
  wire  bypassNetwork_clock; // @[BypassNetwork.scala 114:13]
  wire  bypassNetwork_reset; // @[BypassNetwork.scala 114:13]
  wire  bypassNetwork_io_hold; // @[BypassNetwork.scala 114:13]
  wire [63:0] bypassNetwork_io_source_0; // @[BypassNetwork.scala 114:13]
  wire [63:0] bypassNetwork_io_source_1; // @[BypassNetwork.scala 114:13]
  wire [63:0] bypassNetwork_io_source_2; // @[BypassNetwork.scala 114:13]
  wire [63:0] bypassNetwork_io_target_0; // @[BypassNetwork.scala 114:13]
  wire [63:0] bypassNetwork_io_target_1; // @[BypassNetwork.scala 114:13]
  wire [63:0] bypassNetwork_io_target_2; // @[BypassNetwork.scala 114:13]
  wire  bypassNetwork_io_bypass_0_valid_0; // @[BypassNetwork.scala 114:13]
  wire  bypassNetwork_io_bypass_0_valid_1; // @[BypassNetwork.scala 114:13]
  wire  bypassNetwork_io_bypass_0_valid_2; // @[BypassNetwork.scala 114:13]
  wire [63:0] bypassNetwork_io_bypass_0_data; // @[BypassNetwork.scala 114:13]
  wire [7:0] s0_allocatePtrOH_0 = select_io_allocate_0_bits; // @[ReservationStation.scala 273:{33,33}]
  reg [7:0] validAfterAllocate; // @[ReservationStation.scala 282:35]
  wire  _s0_doEnqueue_0_T = io_fromDispatch_0_ready & io_fromDispatch_0_valid; // @[Decoupled.scala 50:35]
  wire  _s0_doEnqueue_0_T_1 = ~io_redirect_valid; // @[ReservationStation.scala 336:51]
  wire  s0_doEnqueue_0 = _s0_doEnqueue_0_T & ~io_redirect_valid; // @[ReservationStation.scala 336:48]
  wire [7:0] validUpdateByAllocate = s0_doEnqueue_0 ? s0_allocatePtrOH_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire  _numEmptyEntries_T_8 = ~statusArray_io_isValid[0]; // @[ReservationStation.scala 311:76]
  wire  _numEmptyEntries_T_9 = ~statusArray_io_isValid[1]; // @[ReservationStation.scala 311:76]
  wire  _numEmptyEntries_T_10 = ~statusArray_io_isValid[2]; // @[ReservationStation.scala 311:76]
  wire  _numEmptyEntries_T_11 = ~statusArray_io_isValid[3]; // @[ReservationStation.scala 311:76]
  wire  _numEmptyEntries_T_12 = ~statusArray_io_isValid[4]; // @[ReservationStation.scala 311:76]
  wire  _numEmptyEntries_T_13 = ~statusArray_io_isValid[5]; // @[ReservationStation.scala 311:76]
  wire  _numEmptyEntries_T_14 = ~statusArray_io_isValid[6]; // @[ReservationStation.scala 311:76]
  wire  _numEmptyEntries_T_15 = ~statusArray_io_isValid[7]; // @[ReservationStation.scala 311:76]
  wire [1:0] _numEmptyEntries_T_16 = _numEmptyEntries_T_8 + _numEmptyEntries_T_9; // @[Bitwise.scala 48:55]
  wire [1:0] _numEmptyEntries_T_18 = _numEmptyEntries_T_10 + _numEmptyEntries_T_11; // @[Bitwise.scala 48:55]
  wire [2:0] _numEmptyEntries_T_20 = _numEmptyEntries_T_16 + _numEmptyEntries_T_18; // @[Bitwise.scala 48:55]
  wire [1:0] _numEmptyEntries_T_22 = _numEmptyEntries_T_12 + _numEmptyEntries_T_13; // @[Bitwise.scala 48:55]
  wire [1:0] _numEmptyEntries_T_24 = _numEmptyEntries_T_14 + _numEmptyEntries_T_15; // @[Bitwise.scala 48:55]
  wire [2:0] _numEmptyEntries_T_26 = _numEmptyEntries_T_22 + _numEmptyEntries_T_24; // @[Bitwise.scala 48:55]
  wire [3:0] numEmptyEntries = _numEmptyEntries_T_20 + _numEmptyEntries_T_26; // @[Bitwise.scala 48:55]
  wire [3:0] _GEN_477 = {{3'd0}, statusArray_io_update_0_enable}; // @[ReservationStation.scala 313:47]
  wire [3:0] realNumEmptyAfterS1 = numEmptyEntries - _GEN_477; // @[ReservationStation.scala 313:47]
  wire [1:0] highBits = realNumEmptyAfterS1[3:2]; // @[ReservationStation.scala 315:41]
  wire [2:0] numEmptyAfterS1 = |highBits ? 3'h4 : {{1'd0}, realNumEmptyAfterS1[1:0]}; // @[ReservationStation.scala 316:27]
  wire  _numDeq_T = statusArray_io_deqResp_0_valid & statusArray_io_deqResp_0_bits_success; // @[ReservationStation.scala 317:81]
  wire  _numDeq_T_1 = statusArray_io_deqResp_1_valid & statusArray_io_deqResp_1_bits_success; // @[ReservationStation.scala 317:81]
  wire  _numDeq_T_2 = statusArray_io_deqResp_2_valid & statusArray_io_deqResp_2_bits_success; // @[ReservationStation.scala 317:81]
  wire [1:0] _numDeq_T_3 = _numDeq_T_1 + _numDeq_T_2; // @[Bitwise.scala 48:55]
  wire [1:0] _GEN_478 = {{1'd0}, _numDeq_T}; // @[Bitwise.scala 48:55]
  wire [2:0] _numDeq_T_5 = _GEN_478 + _numDeq_T_3; // @[Bitwise.scala 48:55]
  wire [1:0] numDeq = _numDeq_T_5[1:0]; // @[Bitwise.scala 48:55]
  reg [2:0] emptyThisCycle; // @[ReservationStation.scala 318:29]
  wire [2:0] _GEN_479 = {{1'd0}, numDeq}; // @[ReservationStation.scala 319:39]
  reg [1:0] allocateThisCycle; // @[ReservationStation.scala 322:34]
  wire [2:0] _GEN_480 = {{1'd0}, allocateThisCycle}; // @[ReservationStation.scala 324:42]
  wire  pdestMatch = io_slowPorts_0_bits_uop_pdest == io_fromDispatch_0_bits_psrc_0; // @[Bundle.scala 262:30]
  wire  stateCond = pdestMatch & io_slowPorts_0_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond = pdestMatch & (io_slowPorts_0_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_0[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_1 = io_slowPorts_0_bits_uop_pdest == io_fromDispatch_0_bits_psrc_1; // @[Bundle.scala 262:30]
  wire  stateCond_1 = pdestMatch_1 & io_slowPorts_0_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_1 = pdestMatch_1 & (io_slowPorts_0_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_1[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_2 = io_slowPorts_0_bits_uop_pdest == io_fromDispatch_0_bits_psrc_2; // @[Bundle.scala 262:30]
  wire  stateCond_2 = pdestMatch_2 & io_slowPorts_0_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_2 = pdestMatch_2 & (io_slowPorts_0_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_2[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_3 = io_slowPorts_1_bits_uop_pdest == io_fromDispatch_0_bits_psrc_0; // @[Bundle.scala 262:30]
  wire  stateCond_3 = pdestMatch_3 & io_slowPorts_1_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_3 = pdestMatch_3 & (io_slowPorts_1_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_0[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_4 = io_slowPorts_1_bits_uop_pdest == io_fromDispatch_0_bits_psrc_1; // @[Bundle.scala 262:30]
  wire  stateCond_4 = pdestMatch_4 & io_slowPorts_1_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_4 = pdestMatch_4 & (io_slowPorts_1_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_1[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_5 = io_slowPorts_1_bits_uop_pdest == io_fromDispatch_0_bits_psrc_2; // @[Bundle.scala 262:30]
  wire  stateCond_5 = pdestMatch_5 & io_slowPorts_1_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_5 = pdestMatch_5 & (io_slowPorts_1_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_2[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_6 = io_slowPorts_2_bits_uop_pdest == io_fromDispatch_0_bits_psrc_0; // @[Bundle.scala 262:30]
  wire  stateCond_6 = pdestMatch_6 & io_slowPorts_2_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_6 = pdestMatch_6 & (io_slowPorts_2_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_0[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_7 = io_slowPorts_2_bits_uop_pdest == io_fromDispatch_0_bits_psrc_1; // @[Bundle.scala 262:30]
  wire  stateCond_7 = pdestMatch_7 & io_slowPorts_2_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_7 = pdestMatch_7 & (io_slowPorts_2_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_1[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_8 = io_slowPorts_2_bits_uop_pdest == io_fromDispatch_0_bits_psrc_2; // @[Bundle.scala 262:30]
  wire  stateCond_8 = pdestMatch_8 & io_slowPorts_2_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_8 = pdestMatch_8 & (io_slowPorts_2_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_2[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_9 = io_slowPorts_3_bits_uop_pdest == io_fromDispatch_0_bits_psrc_0; // @[Bundle.scala 262:30]
  wire  stateCond_9 = pdestMatch_9 & io_slowPorts_3_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_9 = pdestMatch_9 & (io_slowPorts_3_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_0[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_10 = io_slowPorts_3_bits_uop_pdest == io_fromDispatch_0_bits_psrc_1; // @[Bundle.scala 262:30]
  wire  stateCond_10 = pdestMatch_10 & io_slowPorts_3_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_10 = pdestMatch_10 & (io_slowPorts_3_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_1[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_11 = io_slowPorts_3_bits_uop_pdest == io_fromDispatch_0_bits_psrc_2; // @[Bundle.scala 262:30]
  wire  stateCond_11 = pdestMatch_11 & io_slowPorts_3_bits_uop_ctrl_fpWen; // @[Bundle.scala 268:34]
  wire  dataCond_11 = pdestMatch_11 & (io_slowPorts_3_bits_uop_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_2[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_12 = io_fastUopsIn_0_bits_pdest == io_fromDispatch_0_bits_psrc_0; // @[Bundle.scala 262:30]
  wire  dataCond_12 = pdestMatch_12 & (io_fastUopsIn_0_bits_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_0[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_13 = io_fastUopsIn_0_bits_pdest == io_fromDispatch_0_bits_psrc_1; // @[Bundle.scala 262:30]
  wire  dataCond_13 = pdestMatch_13 & (io_fastUopsIn_0_bits_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_1[1]); // @[Bundle.scala 271:33]
  wire  pdestMatch_14 = io_fastUopsIn_0_bits_pdest == io_fromDispatch_0_bits_psrc_2; // @[Bundle.scala 262:30]
  wire  dataCond_14 = pdestMatch_14 & (io_fastUopsIn_0_bits_ctrl_fpWen & io_fromDispatch_0_bits_ctrl_srcType_2[1]); // @[Bundle.scala 271:33]
  wire  _s0_enqWakeup_0_0_T = io_slowPorts_0_valid & stateCond; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_0_T_1 = io_slowPorts_1_valid & stateCond_3; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_0_T_2 = io_slowPorts_2_valid & stateCond_6; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_0_T_3 = io_slowPorts_3_valid & stateCond_9; // @[ReservationStation.scala 341:90]
  wire [1:0] s0_enqWakeup_0_0_lo = {_s0_enqWakeup_0_0_T_1,_s0_enqWakeup_0_0_T}; // @[ReservationStation.scala 341:100]
  wire [1:0] s0_enqWakeup_0_0_hi = {_s0_enqWakeup_0_0_T_3,_s0_enqWakeup_0_0_T_2}; // @[ReservationStation.scala 341:100]
  wire  _s0_enqDataCapture_0_0_T = io_slowPorts_0_valid & dataCond; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_0_T_1 = io_slowPorts_1_valid & dataCond_3; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_0_T_2 = io_slowPorts_2_valid & dataCond_6; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_0_T_3 = io_slowPorts_3_valid & dataCond_9; // @[ReservationStation.scala 342:94]
  wire [1:0] s0_enqDataCapture_0_0_lo = {_s0_enqDataCapture_0_0_T_1,_s0_enqDataCapture_0_0_T}; // @[ReservationStation.scala 342:104]
  wire [1:0] s0_enqDataCapture_0_0_hi = {_s0_enqDataCapture_0_0_T_3,_s0_enqDataCapture_0_0_T_2}; // @[ReservationStation.scala 342:104]
  wire  _s0_enqWakeup_0_1_T = io_slowPorts_0_valid & stateCond_1; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_1_T_1 = io_slowPorts_1_valid & stateCond_4; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_1_T_2 = io_slowPorts_2_valid & stateCond_7; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_1_T_3 = io_slowPorts_3_valid & stateCond_10; // @[ReservationStation.scala 341:90]
  wire [1:0] s0_enqWakeup_0_1_lo = {_s0_enqWakeup_0_1_T_1,_s0_enqWakeup_0_1_T}; // @[ReservationStation.scala 341:100]
  wire [1:0] s0_enqWakeup_0_1_hi = {_s0_enqWakeup_0_1_T_3,_s0_enqWakeup_0_1_T_2}; // @[ReservationStation.scala 341:100]
  wire  _s0_enqDataCapture_0_1_T = io_slowPorts_0_valid & dataCond_1; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_1_T_1 = io_slowPorts_1_valid & dataCond_4; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_1_T_2 = io_slowPorts_2_valid & dataCond_7; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_1_T_3 = io_slowPorts_3_valid & dataCond_10; // @[ReservationStation.scala 342:94]
  wire [1:0] s0_enqDataCapture_0_1_lo = {_s0_enqDataCapture_0_1_T_1,_s0_enqDataCapture_0_1_T}; // @[ReservationStation.scala 342:104]
  wire [1:0] s0_enqDataCapture_0_1_hi = {_s0_enqDataCapture_0_1_T_3,_s0_enqDataCapture_0_1_T_2}; // @[ReservationStation.scala 342:104]
  wire  _s0_enqWakeup_0_2_T = io_slowPorts_0_valid & stateCond_2; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_2_T_1 = io_slowPorts_1_valid & stateCond_5; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_2_T_2 = io_slowPorts_2_valid & stateCond_8; // @[ReservationStation.scala 341:90]
  wire  _s0_enqWakeup_0_2_T_3 = io_slowPorts_3_valid & stateCond_11; // @[ReservationStation.scala 341:90]
  wire [1:0] s0_enqWakeup_0_2_lo = {_s0_enqWakeup_0_2_T_1,_s0_enqWakeup_0_2_T}; // @[ReservationStation.scala 341:100]
  wire [1:0] s0_enqWakeup_0_2_hi = {_s0_enqWakeup_0_2_T_3,_s0_enqWakeup_0_2_T_2}; // @[ReservationStation.scala 341:100]
  wire  _s0_enqDataCapture_0_2_T = io_slowPorts_0_valid & dataCond_2; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_2_T_1 = io_slowPorts_1_valid & dataCond_5; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_2_T_2 = io_slowPorts_2_valid & dataCond_8; // @[ReservationStation.scala 342:94]
  wire  _s0_enqDataCapture_0_2_T_3 = io_slowPorts_3_valid & dataCond_11; // @[ReservationStation.scala 342:94]
  wire [1:0] s0_enqDataCapture_0_2_lo = {_s0_enqDataCapture_0_2_T_1,_s0_enqDataCapture_0_2_T}; // @[ReservationStation.scala 342:104]
  wire [1:0] s0_enqDataCapture_0_2_hi = {_s0_enqDataCapture_0_2_T_3,_s0_enqDataCapture_0_2_T_2}; // @[ReservationStation.scala 342:104]
  reg [7:0] enqVec_REG; // @[ReservationStation.scala 361:86]
  wire [7:0] _s1_oldestSel_out_valid_T = statusArray_io_canIssue & s1_oldestSel_age_io_out; // @[SelectPolicy.scala 178:28]
  reg  s1_dispatchUops_dup_0_0_valid; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_0_0_bits_ctrl_srcType_0; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_0_0_bits_ctrl_srcType_1; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_0_0_bits_ctrl_srcType_2; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_ctrl_rfWen; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_ctrl_fpWen; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_isAddSub; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_0_0_bits_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_ren3; // @[ReservationStation.scala 391:32]
  reg [2:0] s1_dispatchUops_dup_0_0_bits_ctrl_fpu_rm; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_srcState_0; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_srcState_1; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_srcState_2; // @[ReservationStation.scala 391:32]
  reg [5:0] s1_dispatchUops_dup_0_0_bits_psrc_0; // @[ReservationStation.scala 391:32]
  reg [5:0] s1_dispatchUops_dup_0_0_bits_psrc_1; // @[ReservationStation.scala 391:32]
  reg [5:0] s1_dispatchUops_dup_0_0_bits_psrc_2; // @[ReservationStation.scala 391:32]
  reg [5:0] s1_dispatchUops_dup_0_0_bits_pdest; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_0_0_bits_robIdx_flag; // @[ReservationStation.scala 391:32]
  reg [4:0] s1_dispatchUops_dup_0_0_bits_robIdx_value; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_valid; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_cf_pd_isRVC; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_1_0_bits_cf_pd_brType; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_cf_pd_isCall; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_cf_pd_isRet; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_cf_pred_taken; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_flag; // @[ReservationStation.scala 391:32]
  reg [2:0] s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_value; // @[ReservationStation.scala 391:32]
  reg [2:0] s1_dispatchUops_dup_1_0_bits_cf_ftqOffset; // @[ReservationStation.scala 391:32]
  reg [3:0] s1_dispatchUops_dup_1_0_bits_ctrl_fuType; // @[ReservationStation.scala 391:32]
  reg [6:0] s1_dispatchUops_dup_1_0_bits_ctrl_fuOpType; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_rfWen; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpWen; // @[ReservationStation.scala 391:32]
  reg [19:0] s1_dispatchUops_dup_1_0_bits_ctrl_imm; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_isAddSub; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagOut; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fromInt; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_wflags; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fpWen; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_div; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_sqrt; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fcvt; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typ; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmt; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_ren3; // @[ReservationStation.scala 391:32]
  reg [2:0] s1_dispatchUops_dup_1_0_bits_ctrl_fpu_rm; // @[ReservationStation.scala 391:32]
  reg [5:0] s1_dispatchUops_dup_1_0_bits_pdest; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_1_0_bits_robIdx_flag; // @[ReservationStation.scala 391:32]
  reg [4:0] s1_dispatchUops_dup_1_0_bits_robIdx_value; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_2_0_bits_ctrl_srcType_0; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_2_0_bits_ctrl_srcType_1; // @[ReservationStation.scala 391:32]
  reg [1:0] s1_dispatchUops_dup_2_0_bits_ctrl_srcType_2; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_2_0_bits_srcState_0; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_2_0_bits_srcState_1; // @[ReservationStation.scala 391:32]
  reg  s1_dispatchUops_dup_2_0_bits_srcState_2; // @[ReservationStation.scala 391:32]
  reg [7:0] s1_allocatePtrOH_dup_0_0; // @[ReservationStation.scala 393:37]
  reg [7:0] s1_allocatePtrOH_dup_1_0; // @[ReservationStation.scala 393:37]
  reg [7:0] s1_allocatePtrOH_dup_2_0; // @[ReservationStation.scala 393:37]
  reg [3:0] s1_enqWakeup_0_0; // @[ReservationStation.scala 395:29]
  reg [3:0] s1_enqWakeup_0_1; // @[ReservationStation.scala 395:29]
  reg [3:0] s1_enqWakeup_0_2; // @[ReservationStation.scala 395:29]
  reg [3:0] s1_enqDataCapture_0_0; // @[ReservationStation.scala 396:34]
  reg [3:0] s1_enqDataCapture_0_1; // @[ReservationStation.scala 396:34]
  reg [3:0] s1_enqDataCapture_0_2; // @[ReservationStation.scala 396:34]
  reg  s1_fastWakeup_0_0_0; // @[ReservationStation.scala 397:30]
  reg  s1_fastWakeup_0_1_0; // @[ReservationStation.scala 397:30]
  reg  s1_fastWakeup_0_2_0; // @[ReservationStation.scala 397:30]
  wire  s1_issue_oldest_0 = oldestSelection_io_isOverrided_0; // @[ReservationStation.scala 402:29 504:21]
  wire [7:0] s1_in_oldestPtrOH_bits = s1_oldestSel_age_io_out; // @[SelectPolicy.scala 177:19 179:14]
  wire [7:0] _s1_issuePtrOH_0_bits_T = select_io_grant_0_valid ? select_io_grant_0_bits : s1_allocatePtrOH_dup_0_0; // @[ReservationStation.scala 516:10]
  wire [7:0] s1_issuePtrOH_0_bits = s1_issue_oldest_0 ? s1_in_oldestPtrOH_bits : _s1_issuePtrOH_0_bits_T; // @[ReservationStation.scala 515:33]
  wire [3:0] s1_issuePtr_hi = s1_issuePtrOH_0_bits[7:4]; // @[OneHot.scala 30:18]
  wire [3:0] s1_issuePtr_lo = s1_issuePtrOH_0_bits[3:0]; // @[OneHot.scala 31:18]
  wire  _s1_issuePtr_T = |s1_issuePtr_hi; // @[OneHot.scala 32:14]
  wire [3:0] _s1_issuePtr_T_1 = s1_issuePtr_hi | s1_issuePtr_lo; // @[OneHot.scala 32:28]
  wire [1:0] s1_issuePtr_hi_1 = _s1_issuePtr_T_1[3:2]; // @[OneHot.scala 30:18]
  wire [1:0] s1_issuePtr_lo_1 = _s1_issuePtr_T_1[1:0]; // @[OneHot.scala 31:18]
  wire  _s1_issuePtr_T_2 = |s1_issuePtr_hi_1; // @[OneHot.scala 32:14]
  wire [1:0] _s1_issuePtr_T_3 = s1_issuePtr_hi_1 | s1_issuePtr_lo_1; // @[OneHot.scala 32:28]
  wire [2:0] s1_issuePtr_0 = {_s1_issuePtr_T,_s1_issuePtr_T_2,_s1_issuePtr_T_3[1]}; // @[Cat.scala 31:58]
  wire  _s1_issuePtrOH_0_valid_T = s1_issue_oldest_0 | select_io_grant_0_valid; // @[ReservationStation.scala 514:50]
  wire  _canBypass_WIRE_1 = statusArray_io_update_0_data_srcState_1; // @[StatusArray.scala 61:{13,13}]
  wire  _canBypass_WIRE_0 = statusArray_io_update_0_data_srcState_0; // @[StatusArray.scala 61:{13,13}]
  wire [1:0] _canBypass_T = {_canBypass_WIRE_1,_canBypass_WIRE_0}; // @[StatusArray.scala 61:31]
  wire  canBypass_scheduledCond = ~statusArray_io_update_0_data_scheduled; // @[StatusArray.scala 57:54]
  wire  canBypass_midStateReady = statusArray_io_update_0_data_srcState_2 & statusArray_io_update_0_data_midState; // @[StatusArray.scala 60:63]
  wire  _canBypass_T_3 = &_canBypass_T & canBypass_scheduledCond | canBypass_midStateReady; // @[StatusArray.scala 61:60]
  wire  canBypass = s1_dispatchUops_dup_0_0_valid & _canBypass_T_3; // @[ReservationStation.scala 511:55]
  wire  s1_issuePtrOH_0_valid = s1_issue_oldest_0 | select_io_grant_0_valid | canBypass; // @[ReservationStation.scala 514:77]
  wire  _statusArray_io_update_0_data_srcState_0_T_2 = s1_dispatchUops_dup_0_0_bits_ctrl_srcType_0[0] |
    s1_dispatchUops_dup_0_0_bits_srcState_0; // @[Bundle.scala 245:81]
  wire  _statusArray_io_update_0_data_srcState_0_T_5 = s1_dispatchUops_dup_0_0_bits_ctrl_srcType_1[0] |
    s1_dispatchUops_dup_0_0_bits_srcState_1; // @[Bundle.scala 245:81]
  wire  _statusArray_io_update_0_data_srcState_0_T_8 = s1_dispatchUops_dup_0_0_bits_ctrl_srcType_2[0] |
    s1_dispatchUops_dup_0_0_bits_srcState_2; // @[Bundle.scala 245:81]
  wire  _s1_issue_dispatch_0_T = ~s1_issue_oldest_0; // @[ReservationStation.scala 512:42]
  wire  s1_issue_dispatch_0 = canBypass & ~s1_issue_oldest_0 & ~select_io_grant_0_valid; // @[ReservationStation.scala 512:62]
  reg  valid; // @[PipelineConnect.scala 108:24]
  wire  s2_deq_0_ready = ~valid | io_deq_0_ready; // @[ReservationStation.scala 747:41]
  wire  _statusArray_io_issueGranted_0_valid_T = s1_issue_dispatch_0 & s2_deq_0_ready; // @[ReservationStation.scala 479:84]
  wire  _statusArray_io_issueGranted_1_valid_T_2 = select_io_grant_0_valid & _s1_issue_dispatch_0_T & s2_deq_0_ready; // @[ReservationStation.scala 484:72]
  wire  statusArray_io_issueGranted_2_valid_xs_0 = s1_issue_oldest_0 & s2_deq_0_ready; // @[ParallelMux.scala 64:44]
  wire  _s1_out_0_bits_uop_T_robIdx_flag = select_io_grant_0_valid ? payloadArray_io_read_0_data_robIdx_flag :
    s1_dispatchUops_dup_0_0_bits_robIdx_flag; // @[ReservationStation.scala 519:10]
  wire [4:0] _s1_out_0_bits_uop_T_robIdx_value = select_io_grant_0_valid ? payloadArray_io_read_0_data_robIdx_value :
    s1_dispatchUops_dup_0_0_bits_robIdx_value; // @[ReservationStation.scala 519:10]
  wire  s1_out_0_bits_uop_robIdx_flag = s1_issue_oldest_0 ? payloadArray_io_read_1_data_robIdx_flag :
    _s1_out_0_bits_uop_T_robIdx_flag; // @[ReservationStation.scala 518:30]
  wire [4:0] s1_out_0_bits_uop_robIdx_value = s1_issue_oldest_0 ? payloadArray_io_read_1_data_robIdx_value :
    _s1_out_0_bits_uop_T_robIdx_value; // @[ReservationStation.scala 518:30]
  wire [2:0] _s1_all_src_ready_0_T = {statusArray_io_update_0_data_srcState_2,statusArray_io_update_0_data_srcState_1,
    statusArray_io_update_0_data_srcState_0}; // @[StatusArray.scala 66:14]
  wire  _s1_all_src_ready_0_T_2 = &_s1_all_src_ready_0_T | canBypass_midStateReady; // @[StatusArray.scala 66:26]
  wire  _s1_all_src_ready_0_T_3 = select_io_grant_0_valid ? statusArray_io_allSrcReady_1 : _s1_all_src_ready_0_T_2; // @[ReservationStation.scala 524:12]
  wire  s1_all_src_ready_0 = s1_issue_oldest_0 ? statusArray_io_allSrcReady_2 : _s1_all_src_ready_0_T_3; // @[ReservationStation.scala 523:31]
  wire [5:0] _s1_out_0_valid_flushItself_T_1 = {s1_out_0_bits_uop_robIdx_flag,s1_out_0_bits_uop_robIdx_value}; // @[CircularQueuePtr.scala 61:50]
  wire [5:0] _s1_out_0_valid_flushItself_T_2 = {io_redirect_bits_robIdx_flag,io_redirect_bits_robIdx_value}; // @[CircularQueuePtr.scala 61:70]
  wire  _s1_out_0_valid_flushItself_T_3 = _s1_out_0_valid_flushItself_T_1 == _s1_out_0_valid_flushItself_T_2; // @[CircularQueuePtr.scala 61:52]
  wire  s1_out_0_valid_flushItself = io_redirect_bits_level & _s1_out_0_valid_flushItself_T_3; // @[Rob.scala 122:51]
  wire  s1_out_0_valid_differentFlag = s1_out_0_bits_uop_robIdx_flag ^ io_redirect_bits_robIdx_flag; // @[CircularQueuePtr.scala 86:35]
  wire  s1_out_0_valid_compare = s1_out_0_bits_uop_robIdx_value > io_redirect_bits_robIdx_value; // @[CircularQueuePtr.scala 87:30]
  wire  _s1_out_0_valid_T = s1_out_0_valid_differentFlag ^ s1_out_0_valid_compare; // @[CircularQueuePtr.scala 88:19]
  wire  _s1_out_0_valid_T_2 = io_redirect_valid & (s1_out_0_valid_flushItself | _s1_out_0_valid_T); // @[Rob.scala 123:20]
  wire  s1_out_0_valid = s1_issuePtrOH_0_valid & ~_s1_out_0_valid_T_2; // @[ReservationStation.scala 532:47]
  reg [3:0] slowWakeupMatchVec_0_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_0_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_0_2; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_1_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_1_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_1_2; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_2_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_2_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_2_2; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_3_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_3_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_3_2; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_4_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_4_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_4_2; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_5_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_5_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_5_2; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_6_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_6_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_6_2; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_7_0; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_7_1; // @[ReservationStation.scala 626:31]
  reg [3:0] slowWakeupMatchVec_7_2; // @[ReservationStation.scala 626:31]
  reg  dataArray_io_multiWrite_0_enable_REG; // @[ReservationStation.scala 633:24]
  wire  allocateValid_0 = s1_enqDataCapture_0_0[0] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture = allocateValid_0 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_0_addr_0_T_8 = {slowWakeupMatchVec_7_0[0],slowWakeupMatchVec_6_0[0],
    slowWakeupMatchVec_5_0[0],slowWakeupMatchVec_4_0[0],slowWakeupMatchVec_3_0[0],slowWakeupMatchVec_2_0[0],
    slowWakeupMatchVec_1_0[0],slowWakeupMatchVec_0_0[0]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_1 = s1_enqDataCapture_0_1[0] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_1 = allocateValid_0_1 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_0_addr_1_T_8 = {slowWakeupMatchVec_7_1[0],slowWakeupMatchVec_6_1[0],
    slowWakeupMatchVec_5_1[0],slowWakeupMatchVec_4_1[0],slowWakeupMatchVec_3_1[0],slowWakeupMatchVec_2_1[0],
    slowWakeupMatchVec_1_1[0],slowWakeupMatchVec_0_1[0]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_2 = s1_enqDataCapture_0_2[0] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_2 = allocateValid_0_2 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_0_addr_2_T_8 = {slowWakeupMatchVec_7_2[0],slowWakeupMatchVec_6_2[0],
    slowWakeupMatchVec_5_2[0],slowWakeupMatchVec_4_2[0],slowWakeupMatchVec_3_2[0],slowWakeupMatchVec_2_2[0],
    slowWakeupMatchVec_1_2[0],slowWakeupMatchVec_0_2[0]}; // @[ReservationStation.scala 637:61]
  reg [63:0] dataArray_io_multiWrite_0_data_r; // @[Reg.scala 16:16]
  reg  dataArray_io_multiWrite_1_enable_REG; // @[ReservationStation.scala 633:24]
  wire  allocateValid_0_3 = s1_enqDataCapture_0_0[1] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_3 = allocateValid_0_3 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_1_addr_0_T_8 = {slowWakeupMatchVec_7_0[1],slowWakeupMatchVec_6_0[1],
    slowWakeupMatchVec_5_0[1],slowWakeupMatchVec_4_0[1],slowWakeupMatchVec_3_0[1],slowWakeupMatchVec_2_0[1],
    slowWakeupMatchVec_1_0[1],slowWakeupMatchVec_0_0[1]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_4 = s1_enqDataCapture_0_1[1] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_4 = allocateValid_0_4 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_1_addr_1_T_8 = {slowWakeupMatchVec_7_1[1],slowWakeupMatchVec_6_1[1],
    slowWakeupMatchVec_5_1[1],slowWakeupMatchVec_4_1[1],slowWakeupMatchVec_3_1[1],slowWakeupMatchVec_2_1[1],
    slowWakeupMatchVec_1_1[1],slowWakeupMatchVec_0_1[1]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_5 = s1_enqDataCapture_0_2[1] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_5 = allocateValid_0_5 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_1_addr_2_T_8 = {slowWakeupMatchVec_7_2[1],slowWakeupMatchVec_6_2[1],
    slowWakeupMatchVec_5_2[1],slowWakeupMatchVec_4_2[1],slowWakeupMatchVec_3_2[1],slowWakeupMatchVec_2_2[1],
    slowWakeupMatchVec_1_2[1],slowWakeupMatchVec_0_2[1]}; // @[ReservationStation.scala 637:61]
  reg [63:0] dataArray_io_multiWrite_1_data_r; // @[Reg.scala 16:16]
  reg  dataArray_io_multiWrite_2_enable_REG; // @[ReservationStation.scala 633:24]
  wire  allocateValid_0_6 = s1_enqDataCapture_0_0[2] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_6 = allocateValid_0_6 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_2_addr_0_T_8 = {slowWakeupMatchVec_7_0[2],slowWakeupMatchVec_6_0[2],
    slowWakeupMatchVec_5_0[2],slowWakeupMatchVec_4_0[2],slowWakeupMatchVec_3_0[2],slowWakeupMatchVec_2_0[2],
    slowWakeupMatchVec_1_0[2],slowWakeupMatchVec_0_0[2]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_7 = s1_enqDataCapture_0_1[2] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_7 = allocateValid_0_7 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_2_addr_1_T_8 = {slowWakeupMatchVec_7_1[2],slowWakeupMatchVec_6_1[2],
    slowWakeupMatchVec_5_1[2],slowWakeupMatchVec_4_1[2],slowWakeupMatchVec_3_1[2],slowWakeupMatchVec_2_1[2],
    slowWakeupMatchVec_1_1[2],slowWakeupMatchVec_0_1[2]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_8 = s1_enqDataCapture_0_2[2] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_8 = allocateValid_0_8 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_2_addr_2_T_8 = {slowWakeupMatchVec_7_2[2],slowWakeupMatchVec_6_2[2],
    slowWakeupMatchVec_5_2[2],slowWakeupMatchVec_4_2[2],slowWakeupMatchVec_3_2[2],slowWakeupMatchVec_2_2[2],
    slowWakeupMatchVec_1_2[2],slowWakeupMatchVec_0_2[2]}; // @[ReservationStation.scala 637:61]
  reg [63:0] dataArray_io_multiWrite_2_data_r; // @[Reg.scala 16:16]
  reg  dataArray_io_multiWrite_3_enable_REG; // @[ReservationStation.scala 633:24]
  wire  allocateValid_0_9 = s1_enqDataCapture_0_0[3] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_9 = allocateValid_0_9 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_3_addr_0_T_8 = {slowWakeupMatchVec_7_0[3],slowWakeupMatchVec_6_0[3],
    slowWakeupMatchVec_5_0[3],slowWakeupMatchVec_4_0[3],slowWakeupMatchVec_3_0[3],slowWakeupMatchVec_2_0[3],
    slowWakeupMatchVec_1_0[3],slowWakeupMatchVec_0_0[3]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_10 = s1_enqDataCapture_0_1[3] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_10 = allocateValid_0_10 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_3_addr_1_T_8 = {slowWakeupMatchVec_7_1[3],slowWakeupMatchVec_6_1[3],
    slowWakeupMatchVec_5_1[3],slowWakeupMatchVec_4_1[3],slowWakeupMatchVec_3_1[3],slowWakeupMatchVec_2_1[3],
    slowWakeupMatchVec_1_1[3],slowWakeupMatchVec_0_1[3]}; // @[ReservationStation.scala 637:61]
  wire  allocateValid_0_11 = s1_enqDataCapture_0_2[3] & s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 635:93]
  wire [7:0] allocateDataCapture_11 = allocateValid_0_11 ? s1_allocatePtrOH_dup_2_0 : 8'h0; // @[ParallelMux.scala 64:44]
  wire [7:0] _dataArray_io_multiWrite_3_addr_2_T_8 = {slowWakeupMatchVec_7_2[3],slowWakeupMatchVec_6_2[3],
    slowWakeupMatchVec_5_2[3],slowWakeupMatchVec_4_2[3],slowWakeupMatchVec_3_2[3],slowWakeupMatchVec_2_2[3],
    slowWakeupMatchVec_1_2[3],slowWakeupMatchVec_0_2[3]}; // @[ReservationStation.scala 637:61]
  reg [63:0] dataArray_io_multiWrite_3_data_r; // @[Reg.scala 16:16]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_0_T = dataArray_io_read_0_addr & dataArray_io_multiWrite_0_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_0_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_0_0_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_0_T_3 = dataArray_io_read_0_addr & dataArray_io_multiWrite_1_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_0_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_0_0_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_0_T_6 = dataArray_io_read_0_addr & dataArray_io_multiWrite_2_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_0_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_0_0_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_0_T_9 = dataArray_io_read_0_addr & dataArray_io_multiWrite_3_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_0_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_0_0_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_0_0_lo = {_dataSelect_io_fromSlowPorts_0_0_T_5,
    _dataSelect_io_fromSlowPorts_0_0_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_0_0_hi = {_dataSelect_io_fromSlowPorts_0_0_T_11,
    _dataSelect_io_fromSlowPorts_0_0_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_1_T = dataArray_io_read_0_addr & dataArray_io_multiWrite_0_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_1_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_0_1_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_1_T_3 = dataArray_io_read_0_addr & dataArray_io_multiWrite_1_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_1_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_0_1_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_1_T_6 = dataArray_io_read_0_addr & dataArray_io_multiWrite_2_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_1_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_0_1_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_1_T_9 = dataArray_io_read_0_addr & dataArray_io_multiWrite_3_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_1_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_0_1_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_0_1_lo = {_dataSelect_io_fromSlowPorts_0_1_T_5,
    _dataSelect_io_fromSlowPorts_0_1_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_0_1_hi = {_dataSelect_io_fromSlowPorts_0_1_T_11,
    _dataSelect_io_fromSlowPorts_0_1_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_2_T = dataArray_io_read_0_addr & dataArray_io_multiWrite_0_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_2_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_0_2_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_2_T_3 = dataArray_io_read_0_addr & dataArray_io_multiWrite_1_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_2_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_0_2_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_2_T_6 = dataArray_io_read_0_addr & dataArray_io_multiWrite_2_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_2_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_0_2_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_0_2_T_9 = dataArray_io_read_0_addr & dataArray_io_multiWrite_3_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_0_2_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_0_2_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_0_2_lo = {_dataSelect_io_fromSlowPorts_0_2_T_5,
    _dataSelect_io_fromSlowPorts_0_2_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_0_2_hi = {_dataSelect_io_fromSlowPorts_0_2_T_11,
    _dataSelect_io_fromSlowPorts_0_2_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_0_T = dataArray_io_read_1_addr & dataArray_io_multiWrite_0_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_0_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_1_0_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_0_T_3 = dataArray_io_read_1_addr & dataArray_io_multiWrite_1_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_0_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_1_0_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_0_T_6 = dataArray_io_read_1_addr & dataArray_io_multiWrite_2_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_0_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_1_0_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_0_T_9 = dataArray_io_read_1_addr & dataArray_io_multiWrite_3_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_0_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_1_0_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_1_0_lo = {_dataSelect_io_fromSlowPorts_1_0_T_5,
    _dataSelect_io_fromSlowPorts_1_0_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_1_0_hi = {_dataSelect_io_fromSlowPorts_1_0_T_11,
    _dataSelect_io_fromSlowPorts_1_0_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_1_T = dataArray_io_read_1_addr & dataArray_io_multiWrite_0_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_1_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_1_1_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_1_T_3 = dataArray_io_read_1_addr & dataArray_io_multiWrite_1_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_1_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_1_1_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_1_T_6 = dataArray_io_read_1_addr & dataArray_io_multiWrite_2_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_1_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_1_1_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_1_T_9 = dataArray_io_read_1_addr & dataArray_io_multiWrite_3_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_1_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_1_1_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_1_1_lo = {_dataSelect_io_fromSlowPorts_1_1_T_5,
    _dataSelect_io_fromSlowPorts_1_1_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_1_1_hi = {_dataSelect_io_fromSlowPorts_1_1_T_11,
    _dataSelect_io_fromSlowPorts_1_1_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_2_T = dataArray_io_read_1_addr & dataArray_io_multiWrite_0_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_2_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_1_2_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_2_T_3 = dataArray_io_read_1_addr & dataArray_io_multiWrite_1_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_2_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_1_2_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_2_T_6 = dataArray_io_read_1_addr & dataArray_io_multiWrite_2_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_2_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_1_2_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_1_2_T_9 = dataArray_io_read_1_addr & dataArray_io_multiWrite_3_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_1_2_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_1_2_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_1_2_lo = {_dataSelect_io_fromSlowPorts_1_2_T_5,
    _dataSelect_io_fromSlowPorts_1_2_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_1_2_hi = {_dataSelect_io_fromSlowPorts_1_2_T_11,
    _dataSelect_io_fromSlowPorts_1_2_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_0_T = dataArray_io_write_0_addr & dataArray_io_multiWrite_0_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_0_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_2_0_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_0_T_3 = dataArray_io_write_0_addr & dataArray_io_multiWrite_1_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_0_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_2_0_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_0_T_6 = dataArray_io_write_0_addr & dataArray_io_multiWrite_2_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_0_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_2_0_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_0_T_9 = dataArray_io_write_0_addr & dataArray_io_multiWrite_3_addr_0; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_0_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_2_0_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_2_0_lo = {_dataSelect_io_fromSlowPorts_2_0_T_5,
    _dataSelect_io_fromSlowPorts_2_0_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_2_0_hi = {_dataSelect_io_fromSlowPorts_2_0_T_11,
    _dataSelect_io_fromSlowPorts_2_0_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_1_T = dataArray_io_write_0_addr & dataArray_io_multiWrite_0_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_1_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_2_1_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_1_T_3 = dataArray_io_write_0_addr & dataArray_io_multiWrite_1_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_1_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_2_1_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_1_T_6 = dataArray_io_write_0_addr & dataArray_io_multiWrite_2_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_1_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_2_1_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_1_T_9 = dataArray_io_write_0_addr & dataArray_io_multiWrite_3_addr_1; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_1_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_2_1_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_2_1_lo = {_dataSelect_io_fromSlowPorts_2_1_T_5,
    _dataSelect_io_fromSlowPorts_2_1_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_2_1_hi = {_dataSelect_io_fromSlowPorts_2_1_T_11,
    _dataSelect_io_fromSlowPorts_2_1_T_8}; // @[ReservationStation.scala 697:103]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_2_T = dataArray_io_write_0_addr & dataArray_io_multiWrite_0_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_2_T_2 = dataArray_io_multiWrite_0_enable & |_dataSelect_io_fromSlowPorts_2_2_T; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_2_T_3 = dataArray_io_write_0_addr & dataArray_io_multiWrite_1_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_2_T_5 = dataArray_io_multiWrite_1_enable & |_dataSelect_io_fromSlowPorts_2_2_T_3; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_2_T_6 = dataArray_io_write_0_addr & dataArray_io_multiWrite_2_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_2_T_8 = dataArray_io_multiWrite_2_enable & |_dataSelect_io_fromSlowPorts_2_2_T_6; // @[ReservationStation.scala 697:68]
  wire [7:0] _dataSelect_io_fromSlowPorts_2_2_T_9 = dataArray_io_write_0_addr & dataArray_io_multiWrite_3_addr_2; // @[ReservationStation.scala 697:77]
  wire  _dataSelect_io_fromSlowPorts_2_2_T_11 = dataArray_io_multiWrite_3_enable & |_dataSelect_io_fromSlowPorts_2_2_T_9
    ; // @[ReservationStation.scala 697:68]
  wire [1:0] dataSelect_io_fromSlowPorts_2_2_lo = {_dataSelect_io_fromSlowPorts_2_2_T_5,
    _dataSelect_io_fromSlowPorts_2_2_T_2}; // @[ReservationStation.scala 697:103]
  wire [1:0] dataSelect_io_fromSlowPorts_2_2_hi = {_dataSelect_io_fromSlowPorts_2_2_T_11,
    _dataSelect_io_fromSlowPorts_2_2_T_8}; // @[ReservationStation.scala 697:103]
  reg  fastWakeupMatch_0_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_0_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_0_2_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_1_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_1_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_1_2_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_2_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_2_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_2_2_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_3_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_3_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_3_2_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_4_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_4_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_4_2_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_5_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_5_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_5_2_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_6_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_6_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_6_2_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_7_0_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_7_1_0; // @[ReservationStation.scala 718:28]
  reg  fastWakeupMatch_7_2_0; // @[ReservationStation.scala 718:28]
  wire  s1_out_fire_0 = s1_out_0_valid & s2_deq_0_ready; // @[ReservationStation.scala 728:60]
  reg [2:0] s2_issuePtr_0; // @[Reg.scala 16:16]
  reg [7:0] s2_issuePtrOH_0; // @[Reg.scala 16:16]
  reg  s2_first_issue_0; // @[Reg.scala 16:16]
  reg  s2_all_src_ready_0; // @[Reg.scala 16:16]
  reg  data_uop_robIdx_flag; // @[Reg.scala 16:16]
  reg [4:0] data_uop_robIdx_value; // @[Reg.scala 16:16]
  wire [5:0] _flushItself_T_1 = {data_uop_robIdx_flag,data_uop_robIdx_value}; // @[CircularQueuePtr.scala 61:50]
  wire  _flushItself_T_3 = _flushItself_T_1 == _s1_out_0_valid_flushItself_T_2; // @[CircularQueuePtr.scala 61:52]
  wire  flushItself = io_redirect_bits_level & _flushItself_T_3; // @[Rob.scala 122:51]
  wire  differentFlag = data_uop_robIdx_flag ^ io_redirect_bits_robIdx_flag; // @[CircularQueuePtr.scala 86:35]
  wire  compare = data_uop_robIdx_value > io_redirect_bits_robIdx_value; // @[CircularQueuePtr.scala 87:30]
  wire  _T_140 = differentFlag ^ compare; // @[CircularQueuePtr.scala 88:19]
  wire  _T_142 = io_redirect_valid & (flushItself | _T_140); // @[Rob.scala 123:20]
  wire  _T_143 = s2_deq_0_ready | _T_142; // @[ReservationStation.scala 736:59]
  wire  _GEN_332 = _T_143 ? 1'h0 : valid; // @[PipelineConnect.scala 108:24 110:{25,33}]
  reg  data_uop_ctrl_rfWen; // @[Reg.scala 16:16]
  reg  data_uop_ctrl_fpWen; // @[Reg.scala 16:16]
  reg  data_uop_ctrl_fpu_isAddSub; // @[Reg.scala 16:16]
  reg  data_uop_ctrl_fpu_typeTagIn; // @[Reg.scala 16:16]
  reg [1:0] data_uop_ctrl_fpu_fmaCmd; // @[Reg.scala 16:16]
  reg  data_uop_ctrl_fpu_ren3; // @[Reg.scala 16:16]
  reg [2:0] data_uop_ctrl_fpu_rm; // @[Reg.scala 16:16]
  reg [5:0] data_uop_pdest; // @[Reg.scala 16:16]
  reg  midFinished2T2_0; // @[ReservationStation.scala 816:57]
  reg [2:0] midIssuePtrT2_0; // @[Reg.scala 16:16]
  wire  cond2_0 = midFinished2T2_0 & midIssuePtrT2_0 == s1_issuePtr_0; // @[ReservationStation.scala 872:44]
  wire  _T_144 = io_deq_0_ready & io_deq_0_valid; // @[Decoupled.scala 50:35]
  wire [7:0] normalIssuePtrOH = s1_issue_oldest_0 ? s1_in_oldestPtrOH_bits : select_io_grant_0_bits; // @[ReservationStation.scala 756:33]
  wire  normalFastWakeupMatch_0_0 = normalIssuePtrOH[0] & fastWakeupMatch_0_0_0 | normalIssuePtrOH[1] &
    fastWakeupMatch_1_0_0 | normalIssuePtrOH[2] & fastWakeupMatch_2_0_0 | normalIssuePtrOH[3] & fastWakeupMatch_3_0_0 |
    normalIssuePtrOH[4] & fastWakeupMatch_4_0_0 | normalIssuePtrOH[5] & fastWakeupMatch_5_0_0 | normalIssuePtrOH[6] &
    fastWakeupMatch_6_0_0 | normalIssuePtrOH[7] & fastWakeupMatch_7_0_0; // @[Mux.scala 27:73]
  wire  normalFastWakeupMatch_1_0 = normalIssuePtrOH[0] & fastWakeupMatch_0_1_0 | normalIssuePtrOH[1] &
    fastWakeupMatch_1_1_0 | normalIssuePtrOH[2] & fastWakeupMatch_2_1_0 | normalIssuePtrOH[3] & fastWakeupMatch_3_1_0 |
    normalIssuePtrOH[4] & fastWakeupMatch_4_1_0 | normalIssuePtrOH[5] & fastWakeupMatch_5_1_0 | normalIssuePtrOH[6] &
    fastWakeupMatch_6_1_0 | normalIssuePtrOH[7] & fastWakeupMatch_7_1_0; // @[Mux.scala 27:73]
  wire  normalFastWakeupMatch_2_0 = normalIssuePtrOH[0] & fastWakeupMatch_0_2_0 | normalIssuePtrOH[1] &
    fastWakeupMatch_1_2_0 | normalIssuePtrOH[2] & fastWakeupMatch_2_2_0 | normalIssuePtrOH[3] & fastWakeupMatch_3_2_0 |
    normalIssuePtrOH[4] & fastWakeupMatch_4_2_0 | normalIssuePtrOH[5] & fastWakeupMatch_5_2_0 | normalIssuePtrOH[6] &
    fastWakeupMatch_6_2_0 | normalIssuePtrOH[7] & fastWakeupMatch_7_2_0; // @[Mux.scala 27:73]
  wire  midFinished2_0 = io_fmaMid_0_waitForAdd & _T_144; // @[ReservationStation.scala 797:75]
  wire  midFinished2T0_0 = midFinished2_0 & ~_T_142; // @[ReservationStation.scala 807:9]
  reg [7:0] midIssuePtrOHT1_0; // @[Reg.scala 16:16]
  reg [2:0] midIssuePtrT1_0; // @[Reg.scala 16:16]
  reg  midFinished2T1_0; // @[ReservationStation.scala 811:57]
  reg [7:0] midIssuePtrOHT2_0; // @[Reg.scala 16:16]
  wire [120:0] writeData = {io_fmaMid_0_out_bits_fp_prod_sign,io_fmaMid_0_out_bits_fp_prod_exp,
    io_fmaMid_0_out_bits_fp_prod_sig,io_fmaMid_0_out_bits_inter_flags_isNaN,io_fmaMid_0_out_bits_inter_flags_isInf,
    io_fmaMid_0_out_bits_inter_flags_isInv,1'h0}; // @[ReservationStation.scala 821:49]
  wire [127:0] readData = {io_deq_0_bits_src_1,io_deq_0_bits_src_0}; // @[Cat.scala 31:58]
  wire  cond11_0 = midFinished2T2_0 & midIssuePtrT2_0 == s2_issuePtr_0; // @[ReservationStation.scala 842:45]
  wire  _GEN_450 = io_deq_0_valid ? 1'h0 : midFinished2T2_0; // @[ReservationStation.scala 849:34 818:43 850:49]
  wire  _GEN_451 = cond11_0 ? io_fmaMid_0_out_bits_fp_prod_sign : readData[120]; // @[ReservationStation.scala 844:18 827:32 845:36]
  wire [10:0] _GEN_452 = cond11_0 ? io_fmaMid_0_out_bits_fp_prod_exp : readData[119:109]; // @[ReservationStation.scala 844:18 827:32 845:36]
  wire [104:0] _GEN_453 = cond11_0 ? io_fmaMid_0_out_bits_fp_prod_sig : readData[108:4]; // @[ReservationStation.scala 844:18 827:32 845:36]
  wire  _GEN_454 = cond11_0 ? io_fmaMid_0_out_bits_inter_flags_isNaN : readData[3]; // @[ReservationStation.scala 844:18 827:32 845:36]
  wire  _GEN_455 = cond11_0 ? io_fmaMid_0_out_bits_inter_flags_isInf : readData[2]; // @[ReservationStation.scala 844:18 827:32 845:36]
  wire  _GEN_456 = cond11_0 ? io_fmaMid_0_out_bits_inter_flags_isInv : readData[1]; // @[ReservationStation.scala 844:18 827:32 845:36]
  wire  _GEN_457 = cond11_0 ? 1'h0 : readData[0]; // @[ReservationStation.scala 844:18 827:32 845:36]
  wire  cond12_0 = cond11_0 & io_deq_0_valid & ~io_deq_0_ready; // @[ReservationStation.scala 857:52]
  wire  hasCond12 = |cond12_0; // @[ReservationStation.scala 858:46]
  reg  hasCond12Reg; // @[ReservationStation.scala 859:33]
  wire  _GEN_459 = io_deq_0_ready ? 1'h0 : hasCond12Reg; // @[ReservationStation.scala 862:36 863:22 859:33]
  reg  io_fmaMid_0_in_bits_rfp_prod_sign; // @[Reg.scala 16:16]
  reg [10:0] io_fmaMid_0_in_bits_rfp_prod_exp; // @[Reg.scala 16:16]
  reg [104:0] io_fmaMid_0_in_bits_rfp_prod_sig; // @[Reg.scala 16:16]
  reg  io_fmaMid_0_in_bits_rinter_flags_isNaN; // @[Reg.scala 16:16]
  reg  io_fmaMid_0_in_bits_rinter_flags_isInf; // @[Reg.scala 16:16]
  reg  io_fmaMid_0_in_bits_rinter_flags_isInv; // @[Reg.scala 16:16]
  reg  io_perf_0_value_REG; // @[PerfCounterUtils.scala 188:35]
  reg  io_perf_0_value_REG_1; // @[PerfCounterUtils.scala 188:27]
  StatusArray_6 statusArray ( // @[ReservationStation.scala 261:27]
    .clock(statusArray_clock),
    .reset(statusArray_reset),
    .io_redirect_valid(statusArray_io_redirect_valid),
    .io_redirect_bits_robIdx_flag(statusArray_io_redirect_bits_robIdx_flag),
    .io_redirect_bits_robIdx_value(statusArray_io_redirect_bits_robIdx_value),
    .io_redirect_bits_level(statusArray_io_redirect_bits_level),
    .io_isValid(statusArray_io_isValid),
    .io_isValidNext(statusArray_io_isValidNext),
    .io_canIssue(statusArray_io_canIssue),
    .io_flushed(statusArray_io_flushed),
    .io_update_0_enable(statusArray_io_update_0_enable),
    .io_update_0_addr(statusArray_io_update_0_addr),
    .io_update_0_data_scheduled(statusArray_io_update_0_data_scheduled),
    .io_update_0_data_srcState_0(statusArray_io_update_0_data_srcState_0),
    .io_update_0_data_srcState_1(statusArray_io_update_0_data_srcState_1),
    .io_update_0_data_srcState_2(statusArray_io_update_0_data_srcState_2),
    .io_update_0_data_midState(statusArray_io_update_0_data_midState),
    .io_update_0_data_psrc_0(statusArray_io_update_0_data_psrc_0),
    .io_update_0_data_psrc_1(statusArray_io_update_0_data_psrc_1),
    .io_update_0_data_psrc_2(statusArray_io_update_0_data_psrc_2),
    .io_update_0_data_srcType_0(statusArray_io_update_0_data_srcType_0),
    .io_update_0_data_srcType_1(statusArray_io_update_0_data_srcType_1),
    .io_update_0_data_srcType_2(statusArray_io_update_0_data_srcType_2),
    .io_update_0_data_robIdx_flag(statusArray_io_update_0_data_robIdx_flag),
    .io_update_0_data_robIdx_value(statusArray_io_update_0_data_robIdx_value),
    .io_update_0_data_isFirstIssue(statusArray_io_update_0_data_isFirstIssue),
    .io_wakeup_0_valid(statusArray_io_wakeup_0_valid),
    .io_wakeup_0_bits_ctrl_fpWen(statusArray_io_wakeup_0_bits_ctrl_fpWen),
    .io_wakeup_0_bits_pdest(statusArray_io_wakeup_0_bits_pdest),
    .io_wakeup_1_valid(statusArray_io_wakeup_1_valid),
    .io_wakeup_1_bits_ctrl_fpWen(statusArray_io_wakeup_1_bits_ctrl_fpWen),
    .io_wakeup_1_bits_pdest(statusArray_io_wakeup_1_bits_pdest),
    .io_wakeup_2_valid(statusArray_io_wakeup_2_valid),
    .io_wakeup_2_bits_ctrl_fpWen(statusArray_io_wakeup_2_bits_ctrl_fpWen),
    .io_wakeup_2_bits_pdest(statusArray_io_wakeup_2_bits_pdest),
    .io_wakeup_3_valid(statusArray_io_wakeup_3_valid),
    .io_wakeup_3_bits_ctrl_fpWen(statusArray_io_wakeup_3_bits_ctrl_fpWen),
    .io_wakeup_3_bits_pdest(statusArray_io_wakeup_3_bits_pdest),
    .io_wakeup_4_valid(statusArray_io_wakeup_4_valid),
    .io_wakeup_4_bits_ctrl_fpWen(statusArray_io_wakeup_4_bits_ctrl_fpWen),
    .io_wakeup_4_bits_pdest(statusArray_io_wakeup_4_bits_pdest),
    .io_wakeupMatch_0_0(statusArray_io_wakeupMatch_0_0),
    .io_wakeupMatch_0_1(statusArray_io_wakeupMatch_0_1),
    .io_wakeupMatch_0_2(statusArray_io_wakeupMatch_0_2),
    .io_wakeupMatch_1_0(statusArray_io_wakeupMatch_1_0),
    .io_wakeupMatch_1_1(statusArray_io_wakeupMatch_1_1),
    .io_wakeupMatch_1_2(statusArray_io_wakeupMatch_1_2),
    .io_wakeupMatch_2_0(statusArray_io_wakeupMatch_2_0),
    .io_wakeupMatch_2_1(statusArray_io_wakeupMatch_2_1),
    .io_wakeupMatch_2_2(statusArray_io_wakeupMatch_2_2),
    .io_wakeupMatch_3_0(statusArray_io_wakeupMatch_3_0),
    .io_wakeupMatch_3_1(statusArray_io_wakeupMatch_3_1),
    .io_wakeupMatch_3_2(statusArray_io_wakeupMatch_3_2),
    .io_wakeupMatch_4_0(statusArray_io_wakeupMatch_4_0),
    .io_wakeupMatch_4_1(statusArray_io_wakeupMatch_4_1),
    .io_wakeupMatch_4_2(statusArray_io_wakeupMatch_4_2),
    .io_wakeupMatch_5_0(statusArray_io_wakeupMatch_5_0),
    .io_wakeupMatch_5_1(statusArray_io_wakeupMatch_5_1),
    .io_wakeupMatch_5_2(statusArray_io_wakeupMatch_5_2),
    .io_wakeupMatch_6_0(statusArray_io_wakeupMatch_6_0),
    .io_wakeupMatch_6_1(statusArray_io_wakeupMatch_6_1),
    .io_wakeupMatch_6_2(statusArray_io_wakeupMatch_6_2),
    .io_wakeupMatch_7_0(statusArray_io_wakeupMatch_7_0),
    .io_wakeupMatch_7_1(statusArray_io_wakeupMatch_7_1),
    .io_wakeupMatch_7_2(statusArray_io_wakeupMatch_7_2),
    .io_issueGranted_0_valid(statusArray_io_issueGranted_0_valid),
    .io_issueGranted_0_bits(statusArray_io_issueGranted_0_bits),
    .io_issueGranted_1_valid(statusArray_io_issueGranted_1_valid),
    .io_issueGranted_1_bits(statusArray_io_issueGranted_1_bits),
    .io_issueGranted_2_valid(statusArray_io_issueGranted_2_valid),
    .io_issueGranted_2_bits(statusArray_io_issueGranted_2_bits),
    .io_isFirstIssue_1(statusArray_io_isFirstIssue_1),
    .io_isFirstIssue_2(statusArray_io_isFirstIssue_2),
    .io_allSrcReady_1(statusArray_io_allSrcReady_1),
    .io_allSrcReady_2(statusArray_io_allSrcReady_2),
    .io_updateMidState(statusArray_io_updateMidState),
    .io_deqResp_0_valid(statusArray_io_deqResp_0_valid),
    .io_deqResp_0_bits_rsMask(statusArray_io_deqResp_0_bits_rsMask),
    .io_deqResp_0_bits_success(statusArray_io_deqResp_0_bits_success),
    .io_deqResp_1_valid(statusArray_io_deqResp_1_valid),
    .io_deqResp_1_bits_rsMask(statusArray_io_deqResp_1_bits_rsMask),
    .io_deqResp_1_bits_success(statusArray_io_deqResp_1_bits_success),
    .io_deqResp_2_valid(statusArray_io_deqResp_2_valid),
    .io_deqResp_2_bits_rsMask(statusArray_io_deqResp_2_bits_rsMask),
    .io_deqResp_2_bits_success(statusArray_io_deqResp_2_bits_success)
  );
  SelectPolicy_2 select ( // @[ReservationStation.scala 262:22]
    .io_validVec(select_io_validVec),
    .io_allocate_0_bits(select_io_allocate_0_bits),
    .io_request(select_io_request),
    .io_grant_0_valid(select_io_grant_0_valid),
    .io_grant_0_bits(select_io_grant_0_bits)
  );
  DataArray_6 dataArray ( // @[ReservationStation.scala 263:25]
    .clock(dataArray_clock),
    .io_read_0_addr(dataArray_io_read_0_addr),
    .io_read_0_data_0(dataArray_io_read_0_data_0),
    .io_read_0_data_1(dataArray_io_read_0_data_1),
    .io_read_0_data_2(dataArray_io_read_0_data_2),
    .io_read_1_addr(dataArray_io_read_1_addr),
    .io_read_1_data_0(dataArray_io_read_1_data_0),
    .io_read_1_data_1(dataArray_io_read_1_data_1),
    .io_read_1_data_2(dataArray_io_read_1_data_2),
    .io_write_0_enable(dataArray_io_write_0_enable),
    .io_write_0_mask_0(dataArray_io_write_0_mask_0),
    .io_write_0_mask_1(dataArray_io_write_0_mask_1),
    .io_write_0_mask_2(dataArray_io_write_0_mask_2),
    .io_write_0_addr(dataArray_io_write_0_addr),
    .io_write_0_data_0(dataArray_io_write_0_data_0),
    .io_write_0_data_1(dataArray_io_write_0_data_1),
    .io_write_0_data_2(dataArray_io_write_0_data_2),
    .io_multiWrite_0_enable(dataArray_io_multiWrite_0_enable),
    .io_multiWrite_0_addr_0(dataArray_io_multiWrite_0_addr_0),
    .io_multiWrite_0_addr_1(dataArray_io_multiWrite_0_addr_1),
    .io_multiWrite_0_addr_2(dataArray_io_multiWrite_0_addr_2),
    .io_multiWrite_0_data(dataArray_io_multiWrite_0_data),
    .io_multiWrite_1_enable(dataArray_io_multiWrite_1_enable),
    .io_multiWrite_1_addr_0(dataArray_io_multiWrite_1_addr_0),
    .io_multiWrite_1_addr_1(dataArray_io_multiWrite_1_addr_1),
    .io_multiWrite_1_addr_2(dataArray_io_multiWrite_1_addr_2),
    .io_multiWrite_1_data(dataArray_io_multiWrite_1_data),
    .io_multiWrite_2_enable(dataArray_io_multiWrite_2_enable),
    .io_multiWrite_2_addr_0(dataArray_io_multiWrite_2_addr_0),
    .io_multiWrite_2_addr_1(dataArray_io_multiWrite_2_addr_1),
    .io_multiWrite_2_addr_2(dataArray_io_multiWrite_2_addr_2),
    .io_multiWrite_2_data(dataArray_io_multiWrite_2_data),
    .io_multiWrite_3_enable(dataArray_io_multiWrite_3_enable),
    .io_multiWrite_3_addr_0(dataArray_io_multiWrite_3_addr_0),
    .io_multiWrite_3_addr_1(dataArray_io_multiWrite_3_addr_1),
    .io_multiWrite_3_addr_2(dataArray_io_multiWrite_3_addr_2),
    .io_multiWrite_3_data(dataArray_io_multiWrite_3_data),
    .io_partialWrite_0_enable(dataArray_io_partialWrite_0_enable),
    .io_partialWrite_0_addr(dataArray_io_partialWrite_0_addr),
    .io_partialWrite_0_data_0(dataArray_io_partialWrite_0_data_0),
    .io_partialWrite_0_data_1(dataArray_io_partialWrite_0_data_1)
  );
  PayloadArray_2 payloadArray ( // @[ReservationStation.scala 264:28]
    .clock(payloadArray_clock),
    .io_read_0_addr(payloadArray_io_read_0_addr),
    .io_read_0_data_cf_pd_isRVC(payloadArray_io_read_0_data_cf_pd_isRVC),
    .io_read_0_data_cf_pd_brType(payloadArray_io_read_0_data_cf_pd_brType),
    .io_read_0_data_cf_pd_isCall(payloadArray_io_read_0_data_cf_pd_isCall),
    .io_read_0_data_cf_pd_isRet(payloadArray_io_read_0_data_cf_pd_isRet),
    .io_read_0_data_cf_pred_taken(payloadArray_io_read_0_data_cf_pred_taken),
    .io_read_0_data_cf_ftqPtr_flag(payloadArray_io_read_0_data_cf_ftqPtr_flag),
    .io_read_0_data_cf_ftqPtr_value(payloadArray_io_read_0_data_cf_ftqPtr_value),
    .io_read_0_data_cf_ftqOffset(payloadArray_io_read_0_data_cf_ftqOffset),
    .io_read_0_data_ctrl_fuType(payloadArray_io_read_0_data_ctrl_fuType),
    .io_read_0_data_ctrl_fuOpType(payloadArray_io_read_0_data_ctrl_fuOpType),
    .io_read_0_data_ctrl_rfWen(payloadArray_io_read_0_data_ctrl_rfWen),
    .io_read_0_data_ctrl_fpWen(payloadArray_io_read_0_data_ctrl_fpWen),
    .io_read_0_data_ctrl_imm(payloadArray_io_read_0_data_ctrl_imm),
    .io_read_0_data_ctrl_fpu_isAddSub(payloadArray_io_read_0_data_ctrl_fpu_isAddSub),
    .io_read_0_data_ctrl_fpu_typeTagIn(payloadArray_io_read_0_data_ctrl_fpu_typeTagIn),
    .io_read_0_data_ctrl_fpu_typeTagOut(payloadArray_io_read_0_data_ctrl_fpu_typeTagOut),
    .io_read_0_data_ctrl_fpu_fromInt(payloadArray_io_read_0_data_ctrl_fpu_fromInt),
    .io_read_0_data_ctrl_fpu_wflags(payloadArray_io_read_0_data_ctrl_fpu_wflags),
    .io_read_0_data_ctrl_fpu_fpWen(payloadArray_io_read_0_data_ctrl_fpu_fpWen),
    .io_read_0_data_ctrl_fpu_fmaCmd(payloadArray_io_read_0_data_ctrl_fpu_fmaCmd),
    .io_read_0_data_ctrl_fpu_div(payloadArray_io_read_0_data_ctrl_fpu_div),
    .io_read_0_data_ctrl_fpu_sqrt(payloadArray_io_read_0_data_ctrl_fpu_sqrt),
    .io_read_0_data_ctrl_fpu_fcvt(payloadArray_io_read_0_data_ctrl_fpu_fcvt),
    .io_read_0_data_ctrl_fpu_typ(payloadArray_io_read_0_data_ctrl_fpu_typ),
    .io_read_0_data_ctrl_fpu_fmt(payloadArray_io_read_0_data_ctrl_fpu_fmt),
    .io_read_0_data_ctrl_fpu_ren3(payloadArray_io_read_0_data_ctrl_fpu_ren3),
    .io_read_0_data_ctrl_fpu_rm(payloadArray_io_read_0_data_ctrl_fpu_rm),
    .io_read_0_data_pdest(payloadArray_io_read_0_data_pdest),
    .io_read_0_data_robIdx_flag(payloadArray_io_read_0_data_robIdx_flag),
    .io_read_0_data_robIdx_value(payloadArray_io_read_0_data_robIdx_value),
    .io_read_1_addr(payloadArray_io_read_1_addr),
    .io_read_1_data_cf_pd_isRVC(payloadArray_io_read_1_data_cf_pd_isRVC),
    .io_read_1_data_cf_pd_brType(payloadArray_io_read_1_data_cf_pd_brType),
    .io_read_1_data_cf_pd_isCall(payloadArray_io_read_1_data_cf_pd_isCall),
    .io_read_1_data_cf_pd_isRet(payloadArray_io_read_1_data_cf_pd_isRet),
    .io_read_1_data_cf_pred_taken(payloadArray_io_read_1_data_cf_pred_taken),
    .io_read_1_data_cf_ftqPtr_flag(payloadArray_io_read_1_data_cf_ftqPtr_flag),
    .io_read_1_data_cf_ftqPtr_value(payloadArray_io_read_1_data_cf_ftqPtr_value),
    .io_read_1_data_cf_ftqOffset(payloadArray_io_read_1_data_cf_ftqOffset),
    .io_read_1_data_ctrl_fuType(payloadArray_io_read_1_data_ctrl_fuType),
    .io_read_1_data_ctrl_fuOpType(payloadArray_io_read_1_data_ctrl_fuOpType),
    .io_read_1_data_ctrl_rfWen(payloadArray_io_read_1_data_ctrl_rfWen),
    .io_read_1_data_ctrl_fpWen(payloadArray_io_read_1_data_ctrl_fpWen),
    .io_read_1_data_ctrl_imm(payloadArray_io_read_1_data_ctrl_imm),
    .io_read_1_data_ctrl_fpu_isAddSub(payloadArray_io_read_1_data_ctrl_fpu_isAddSub),
    .io_read_1_data_ctrl_fpu_typeTagIn(payloadArray_io_read_1_data_ctrl_fpu_typeTagIn),
    .io_read_1_data_ctrl_fpu_typeTagOut(payloadArray_io_read_1_data_ctrl_fpu_typeTagOut),
    .io_read_1_data_ctrl_fpu_fromInt(payloadArray_io_read_1_data_ctrl_fpu_fromInt),
    .io_read_1_data_ctrl_fpu_wflags(payloadArray_io_read_1_data_ctrl_fpu_wflags),
    .io_read_1_data_ctrl_fpu_fpWen(payloadArray_io_read_1_data_ctrl_fpu_fpWen),
    .io_read_1_data_ctrl_fpu_fmaCmd(payloadArray_io_read_1_data_ctrl_fpu_fmaCmd),
    .io_read_1_data_ctrl_fpu_div(payloadArray_io_read_1_data_ctrl_fpu_div),
    .io_read_1_data_ctrl_fpu_sqrt(payloadArray_io_read_1_data_ctrl_fpu_sqrt),
    .io_read_1_data_ctrl_fpu_fcvt(payloadArray_io_read_1_data_ctrl_fpu_fcvt),
    .io_read_1_data_ctrl_fpu_typ(payloadArray_io_read_1_data_ctrl_fpu_typ),
    .io_read_1_data_ctrl_fpu_fmt(payloadArray_io_read_1_data_ctrl_fpu_fmt),
    .io_read_1_data_ctrl_fpu_ren3(payloadArray_io_read_1_data_ctrl_fpu_ren3),
    .io_read_1_data_ctrl_fpu_rm(payloadArray_io_read_1_data_ctrl_fpu_rm),
    .io_read_1_data_pdest(payloadArray_io_read_1_data_pdest),
    .io_read_1_data_robIdx_flag(payloadArray_io_read_1_data_robIdx_flag),
    .io_read_1_data_robIdx_value(payloadArray_io_read_1_data_robIdx_value),
    .io_write_0_enable(payloadArray_io_write_0_enable),
    .io_write_0_addr(payloadArray_io_write_0_addr),
    .io_write_0_data_cf_pd_isRVC(payloadArray_io_write_0_data_cf_pd_isRVC),
    .io_write_0_data_cf_pd_brType(payloadArray_io_write_0_data_cf_pd_brType),
    .io_write_0_data_cf_pd_isCall(payloadArray_io_write_0_data_cf_pd_isCall),
    .io_write_0_data_cf_pd_isRet(payloadArray_io_write_0_data_cf_pd_isRet),
    .io_write_0_data_cf_pred_taken(payloadArray_io_write_0_data_cf_pred_taken),
    .io_write_0_data_cf_ftqPtr_flag(payloadArray_io_write_0_data_cf_ftqPtr_flag),
    .io_write_0_data_cf_ftqPtr_value(payloadArray_io_write_0_data_cf_ftqPtr_value),
    .io_write_0_data_cf_ftqOffset(payloadArray_io_write_0_data_cf_ftqOffset),
    .io_write_0_data_ctrl_fuType(payloadArray_io_write_0_data_ctrl_fuType),
    .io_write_0_data_ctrl_fuOpType(payloadArray_io_write_0_data_ctrl_fuOpType),
    .io_write_0_data_ctrl_rfWen(payloadArray_io_write_0_data_ctrl_rfWen),
    .io_write_0_data_ctrl_fpWen(payloadArray_io_write_0_data_ctrl_fpWen),
    .io_write_0_data_ctrl_imm(payloadArray_io_write_0_data_ctrl_imm),
    .io_write_0_data_ctrl_fpu_isAddSub(payloadArray_io_write_0_data_ctrl_fpu_isAddSub),
    .io_write_0_data_ctrl_fpu_typeTagIn(payloadArray_io_write_0_data_ctrl_fpu_typeTagIn),
    .io_write_0_data_ctrl_fpu_typeTagOut(payloadArray_io_write_0_data_ctrl_fpu_typeTagOut),
    .io_write_0_data_ctrl_fpu_fromInt(payloadArray_io_write_0_data_ctrl_fpu_fromInt),
    .io_write_0_data_ctrl_fpu_wflags(payloadArray_io_write_0_data_ctrl_fpu_wflags),
    .io_write_0_data_ctrl_fpu_fpWen(payloadArray_io_write_0_data_ctrl_fpu_fpWen),
    .io_write_0_data_ctrl_fpu_fmaCmd(payloadArray_io_write_0_data_ctrl_fpu_fmaCmd),
    .io_write_0_data_ctrl_fpu_div(payloadArray_io_write_0_data_ctrl_fpu_div),
    .io_write_0_data_ctrl_fpu_sqrt(payloadArray_io_write_0_data_ctrl_fpu_sqrt),
    .io_write_0_data_ctrl_fpu_fcvt(payloadArray_io_write_0_data_ctrl_fpu_fcvt),
    .io_write_0_data_ctrl_fpu_typ(payloadArray_io_write_0_data_ctrl_fpu_typ),
    .io_write_0_data_ctrl_fpu_fmt(payloadArray_io_write_0_data_ctrl_fpu_fmt),
    .io_write_0_data_ctrl_fpu_ren3(payloadArray_io_write_0_data_ctrl_fpu_ren3),
    .io_write_0_data_ctrl_fpu_rm(payloadArray_io_write_0_data_ctrl_fpu_rm),
    .io_write_0_data_pdest(payloadArray_io_write_0_data_pdest),
    .io_write_0_data_robIdx_flag(payloadArray_io_write_0_data_robIdx_flag),
    .io_write_0_data_robIdx_value(payloadArray_io_write_0_data_robIdx_value)
  );
  AgeDetector_2 s1_oldestSel_age ( // @[SelectPolicy.scala 174:21]
    .clock(s1_oldestSel_age_clock),
    .reset(s1_oldestSel_age_reset),
    .io_enq_0(s1_oldestSel_age_io_enq_0),
    .io_deq(s1_oldestSel_age_io_deq),
    .io_out(s1_oldestSel_age_io_out)
  );
  OldestSelection_1 oldestSelection ( // @[ReservationStation.scala 499:33]
    .io_oldest_valid(oldestSelection_io_oldest_valid),
    .io_isOverrided_0(oldestSelection_io_isOverrided_0)
  );
  ImmExtractor_4 immExt ( // @[DataArray.scala 161:18]
    .io_data_in_0(immExt_io_data_in_0),
    .io_data_in_1(immExt_io_data_in_1),
    .io_data_in_2(immExt_io_data_in_2),
    .io_data_out_0(immExt_io_data_out_0),
    .io_data_out_1(immExt_io_data_out_1),
    .io_data_out_2(immExt_io_data_out_2)
  );
  DataSelect_6 dataSelect ( // @[ReservationStation.scala 691:26]
    .io_doOverride_0(dataSelect_io_doOverride_0),
    .io_readData_0_0(dataSelect_io_readData_0_0),
    .io_readData_0_1(dataSelect_io_readData_0_1),
    .io_readData_0_2(dataSelect_io_readData_0_2),
    .io_readData_1_0(dataSelect_io_readData_1_0),
    .io_readData_1_1(dataSelect_io_readData_1_1),
    .io_readData_1_2(dataSelect_io_readData_1_2),
    .io_fromSlowPorts_0_0(dataSelect_io_fromSlowPorts_0_0),
    .io_fromSlowPorts_0_1(dataSelect_io_fromSlowPorts_0_1),
    .io_fromSlowPorts_0_2(dataSelect_io_fromSlowPorts_0_2),
    .io_fromSlowPorts_1_0(dataSelect_io_fromSlowPorts_1_0),
    .io_fromSlowPorts_1_1(dataSelect_io_fromSlowPorts_1_1),
    .io_fromSlowPorts_1_2(dataSelect_io_fromSlowPorts_1_2),
    .io_fromSlowPorts_2_0(dataSelect_io_fromSlowPorts_2_0),
    .io_fromSlowPorts_2_1(dataSelect_io_fromSlowPorts_2_1),
    .io_fromSlowPorts_2_2(dataSelect_io_fromSlowPorts_2_2),
    .io_slowData_0(dataSelect_io_slowData_0),
    .io_slowData_1(dataSelect_io_slowData_1),
    .io_slowData_2(dataSelect_io_slowData_2),
    .io_slowData_3(dataSelect_io_slowData_3),
    .io_enqBypass_0_0(dataSelect_io_enqBypass_0_0),
    .io_enqData_0_0_bits(dataSelect_io_enqData_0_0_bits),
    .io_enqData_0_1_bits(dataSelect_io_enqData_0_1_bits),
    .io_enqData_0_2_bits(dataSelect_io_enqData_0_2_bits),
    .io_deqData_0_0(dataSelect_io_deqData_0_0),
    .io_deqData_0_1(dataSelect_io_deqData_0_1),
    .io_deqData_0_2(dataSelect_io_deqData_0_2)
  );
  BypassNetworkRight bypassNetwork ( // @[BypassNetwork.scala 114:13]
    .clock(bypassNetwork_clock),
    .reset(bypassNetwork_reset),
    .io_hold(bypassNetwork_io_hold),
    .io_source_0(bypassNetwork_io_source_0),
    .io_source_1(bypassNetwork_io_source_1),
    .io_source_2(bypassNetwork_io_source_2),
    .io_target_0(bypassNetwork_io_target_0),
    .io_target_1(bypassNetwork_io_target_1),
    .io_target_2(bypassNetwork_io_target_2),
    .io_bypass_0_valid_0(bypassNetwork_io_bypass_0_valid_0),
    .io_bypass_0_valid_1(bypassNetwork_io_bypass_0_valid_1),
    .io_bypass_0_valid_2(bypassNetwork_io_bypass_0_valid_2),
    .io_bypass_0_data(bypassNetwork_io_bypass_0_data)
  );
  assign io_fromDispatch_0_ready = emptyThisCycle > _GEN_480; // @[ReservationStation.scala 324:42]
  assign io_deq_0_valid = valid; // @[PipelineConnect.scala 117:17 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_ctrl_rfWen = data_uop_ctrl_rfWen; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_ctrl_fpWen = data_uop_ctrl_fpWen; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_ctrl_fpu_isAddSub = data_uop_ctrl_fpu_isAddSub; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_ctrl_fpu_typeTagIn = data_uop_ctrl_fpu_typeTagIn; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_ctrl_fpu_fmaCmd = data_uop_ctrl_fpu_fmaCmd; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_ctrl_fpu_ren3 = data_uop_ctrl_fpu_ren3; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_ctrl_fpu_rm = data_uop_ctrl_fpu_rm; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_pdest = data_uop_pdest; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_robIdx_flag = data_uop_robIdx_flag; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_uop_robIdx_value = data_uop_robIdx_value; // @[PipelineConnect.scala 116:16 ReservationStation.scala 266:20]
  assign io_deq_0_bits_src_0 = bypassNetwork_io_target_0; // @[ReservationStation.scala 266:20 772:31]
  assign io_deq_0_bits_src_1 = bypassNetwork_io_target_1; // @[ReservationStation.scala 266:20 772:31]
  assign io_deq_0_bits_src_2 = bypassNetwork_io_target_2; // @[ReservationStation.scala 266:20 772:31]
  assign io_fmaMid_0_in_valid = ~s2_first_issue_0; // @[ReservationStation.scala 743:36]
  assign io_fmaMid_0_in_bits_fp_prod_sign = hasCond12Reg ? io_fmaMid_0_in_bits_rfp_prod_sign : _GEN_451; // @[ReservationStation.scala 865:27 867:34]
  assign io_fmaMid_0_in_bits_fp_prod_exp = hasCond12Reg ? io_fmaMid_0_in_bits_rfp_prod_exp : _GEN_452; // @[ReservationStation.scala 865:27 867:34]
  assign io_fmaMid_0_in_bits_fp_prod_sig = hasCond12Reg ? io_fmaMid_0_in_bits_rfp_prod_sig : _GEN_453; // @[ReservationStation.scala 865:27 867:34]
  assign io_fmaMid_0_in_bits_inter_flags_isNaN = hasCond12Reg ? io_fmaMid_0_in_bits_rinter_flags_isNaN : _GEN_454; // @[ReservationStation.scala 865:27 867:34]
  assign io_fmaMid_0_in_bits_inter_flags_isInf = hasCond12Reg ? io_fmaMid_0_in_bits_rinter_flags_isInf : _GEN_455; // @[ReservationStation.scala 865:27 867:34]
  assign io_fmaMid_0_in_bits_inter_flags_isInv = hasCond12Reg ? io_fmaMid_0_in_bits_rinter_flags_isInv : _GEN_456; // @[ReservationStation.scala 865:27 867:34]
  assign io_fmaMid_0_in_bits_inter_flags_overflow = hasCond12Reg ? 1'h0 : _GEN_457; // @[ReservationStation.scala 865:27 867:34]
  assign io_fmaMid_0_waitForAdd = ~s2_all_src_ready_0; // @[ReservationStation.scala 742:38]
  assign io_perf_0_value = {{5'd0}, io_perf_0_value_REG_1}; // @[PerfCounterUtils.scala 188:17]
  assign statusArray_clock = clock;
  assign statusArray_reset = reset;
  assign statusArray_io_redirect_valid = io_redirect_valid; // @[ReservationStation.scala 437:27]
  assign statusArray_io_redirect_bits_robIdx_flag = io_redirect_bits_robIdx_flag; // @[ReservationStation.scala 437:27]
  assign statusArray_io_redirect_bits_robIdx_value = io_redirect_bits_robIdx_value; // @[ReservationStation.scala 437:27]
  assign statusArray_io_redirect_bits_level = io_redirect_bits_level; // @[ReservationStation.scala 437:27]
  assign statusArray_io_update_0_enable = s1_dispatchUops_dup_0_0_valid; // @[ReservationStation.scala 445:25]
  assign statusArray_io_update_0_addr = s1_allocatePtrOH_dup_0_0; // @[ReservationStation.scala 446:23]
  assign statusArray_io_update_0_data_scheduled = 1'h0; // @[ReservationStation.scala 447:60]
  assign statusArray_io_update_0_data_srcState_0 = _statusArray_io_update_0_data_srcState_0_T_2 | |s1_enqWakeup_0_0 | |
    s1_fastWakeup_0_0_0; // @[ReservationStation.scala 452:96]
  assign statusArray_io_update_0_data_srcState_1 = _statusArray_io_update_0_data_srcState_0_T_5 | |s1_enqWakeup_0_1 | |
    s1_fastWakeup_0_1_0; // @[ReservationStation.scala 452:96]
  assign statusArray_io_update_0_data_srcState_2 = _statusArray_io_update_0_data_srcState_0_T_8 | |s1_enqWakeup_0_2 | |
    s1_fastWakeup_0_2_0; // @[ReservationStation.scala 452:96]
  assign statusArray_io_update_0_data_midState = 1'h0; // @[ReservationStation.scala 454:32]
  assign statusArray_io_update_0_data_psrc_0 = s1_dispatchUops_dup_0_0_bits_psrc_0; // @[ReservationStation.scala 455:28]
  assign statusArray_io_update_0_data_psrc_1 = s1_dispatchUops_dup_0_0_bits_psrc_1; // @[ReservationStation.scala 455:28]
  assign statusArray_io_update_0_data_psrc_2 = s1_dispatchUops_dup_0_0_bits_psrc_2; // @[ReservationStation.scala 455:28]
  assign statusArray_io_update_0_data_srcType_0 = s1_dispatchUops_dup_0_0_bits_ctrl_srcType_0; // @[ReservationStation.scala 456:31]
  assign statusArray_io_update_0_data_srcType_1 = s1_dispatchUops_dup_0_0_bits_ctrl_srcType_1; // @[ReservationStation.scala 456:31]
  assign statusArray_io_update_0_data_srcType_2 = s1_dispatchUops_dup_0_0_bits_ctrl_srcType_2; // @[ReservationStation.scala 456:31]
  assign statusArray_io_update_0_data_robIdx_flag = s1_dispatchUops_dup_0_0_bits_robIdx_flag; // @[ReservationStation.scala 457:30]
  assign statusArray_io_update_0_data_robIdx_value = s1_dispatchUops_dup_0_0_bits_robIdx_value; // @[ReservationStation.scala 457:30]
  assign statusArray_io_update_0_data_isFirstIssue = 1'h1; // @[ReservationStation.scala 463:36]
  assign statusArray_io_wakeup_0_valid = io_fastUopsIn_0_valid; // @[ReservationStation.scala 352:18]
  assign statusArray_io_wakeup_0_bits_ctrl_fpWen = io_fastUopsIn_0_bits_ctrl_fpWen; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_0_bits_pdest = io_fastUopsIn_0_bits_pdest; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_1_valid = io_slowPorts_0_valid; // @[ReservationStation.scala 352:18]
  assign statusArray_io_wakeup_1_bits_ctrl_fpWen = io_slowPorts_0_bits_uop_ctrl_fpWen; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_1_bits_pdest = io_slowPorts_0_bits_uop_pdest; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_2_valid = io_slowPorts_1_valid; // @[ReservationStation.scala 352:18]
  assign statusArray_io_wakeup_2_bits_ctrl_fpWen = io_slowPorts_1_bits_uop_ctrl_fpWen; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_2_bits_pdest = io_slowPorts_1_bits_uop_pdest; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_3_valid = io_slowPorts_2_valid; // @[ReservationStation.scala 352:18]
  assign statusArray_io_wakeup_3_bits_ctrl_fpWen = io_slowPorts_2_bits_uop_ctrl_fpWen; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_3_bits_pdest = io_slowPorts_2_bits_uop_pdest; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_4_valid = io_slowPorts_3_valid; // @[ReservationStation.scala 352:18]
  assign statusArray_io_wakeup_4_bits_ctrl_fpWen = io_slowPorts_3_bits_uop_ctrl_fpWen; // @[ReservationStation.scala 353:17]
  assign statusArray_io_wakeup_4_bits_pdest = io_slowPorts_3_bits_uop_pdest; // @[ReservationStation.scala 353:17]
  assign statusArray_io_issueGranted_0_valid = s1_issue_dispatch_0 & s2_deq_0_ready; // @[ReservationStation.scala 479:84]
  assign statusArray_io_issueGranted_0_bits = s1_allocatePtrOH_dup_0_0; // @[ReservationStation.scala 480:21]
  assign statusArray_io_issueGranted_1_valid = select_io_grant_0_valid & _s1_issue_dispatch_0_T & s2_deq_0_ready; // @[ReservationStation.scala 484:72]
  assign statusArray_io_issueGranted_1_bits = select_io_grant_0_bits; // @[ReservationStation.scala 485:21]
  assign statusArray_io_issueGranted_2_valid = s1_issue_oldest_0 & s2_deq_0_ready; // @[ParallelMux.scala 64:44]
  assign statusArray_io_issueGranted_2_bits = s1_oldestSel_age_io_out; // @[SelectPolicy.scala 177:19 179:14]
  assign statusArray_io_updateMidState = midFinished2_0 ? s2_issuePtrOH_0 : 8'h0; // @[ParallelMux.scala 64:44]
  assign statusArray_io_deqResp_0_valid = _statusArray_io_issueGranted_1_valid_T_2 & s1_all_src_ready_0; // @[ReservationStation.scala 550:110]
  assign statusArray_io_deqResp_0_bits_rsMask = select_io_grant_0_bits; // @[ReservationStation.scala 551:47]
  assign statusArray_io_deqResp_0_bits_success = ~valid | io_deq_0_ready; // @[ReservationStation.scala 747:41]
  assign statusArray_io_deqResp_1_valid = _statusArray_io_issueGranted_0_valid_T & _s1_all_src_ready_0_T_2; // @[ReservationStation.scala 556:86]
  assign statusArray_io_deqResp_1_bits_rsMask = s1_allocatePtrOH_dup_0_0; // @[ReservationStation.scala 557:49]
  assign statusArray_io_deqResp_1_bits_success = ~valid | io_deq_0_ready; // @[ReservationStation.scala 747:41]
  assign statusArray_io_deqResp_2_valid = |s1_issue_oldest_0 & statusArray_io_issueGranted_2_valid_xs_0 &
    statusArray_io_allSrcReady_2; // @[ReservationStation.scala 577:122]
  assign statusArray_io_deqResp_2_bits_rsMask = s1_oldestSel_age_io_out; // @[SelectPolicy.scala 177:19 179:14]
  assign statusArray_io_deqResp_2_bits_success = s1_issue_oldest_0 & s2_deq_0_ready; // @[ParallelMux.scala 64:44]
  assign select_io_validVec = validAfterAllocate; // @[ReservationStation.scala 285:22]
  assign select_io_request = statusArray_io_canIssue; // @[ReservationStation.scala 358:21]
  assign dataArray_clock = clock;
  assign dataArray_io_read_0_addr = select_io_grant_0_bits; // @[ReservationStation.scala 376:31]
  assign dataArray_io_read_1_addr = s1_oldestSel_age_io_out; // @[SelectPolicy.scala 177:19 179:14]
  assign dataArray_io_write_0_enable = s1_dispatchUops_dup_2_0_valid; // @[ReservationStation.scala 603:34]
  assign dataArray_io_write_0_mask_0 = s1_dispatchUops_dup_2_0_bits_ctrl_srcType_0[0] |
    s1_dispatchUops_dup_2_0_bits_srcState_0; // @[Bundle.scala 245:81]
  assign dataArray_io_write_0_mask_1 = s1_dispatchUops_dup_2_0_bits_ctrl_srcType_1[0] |
    s1_dispatchUops_dup_2_0_bits_srcState_1; // @[Bundle.scala 245:81]
  assign dataArray_io_write_0_mask_2 = s1_dispatchUops_dup_2_0_bits_ctrl_srcType_2[0] |
    s1_dispatchUops_dup_2_0_bits_srcState_2; // @[Bundle.scala 245:81]
  assign dataArray_io_write_0_addr = s1_allocatePtrOH_dup_2_0; // @[ReservationStation.scala 605:32]
  assign dataArray_io_write_0_data_0 = immExt_io_data_out_0; // @[ReservationStation.scala 589:29 593:12]
  assign dataArray_io_write_0_data_1 = immExt_io_data_out_1; // @[ReservationStation.scala 589:29 593:12]
  assign dataArray_io_write_0_data_2 = immExt_io_data_out_2; // @[ReservationStation.scala 589:29 593:12]
  assign dataArray_io_multiWrite_0_enable = dataArray_io_multiWrite_0_enable_REG; // @[ReservationStation.scala 633:14]
  assign dataArray_io_multiWrite_0_addr_0 = _dataArray_io_multiWrite_0_addr_0_T_8 | allocateDataCapture; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_0_addr_1 = _dataArray_io_multiWrite_0_addr_1_T_8 | allocateDataCapture_1; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_0_addr_2 = _dataArray_io_multiWrite_0_addr_2_T_8 | allocateDataCapture_2; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_0_data = dataArray_io_multiWrite_0_data_r; // @[ReservationStation.scala 639:12]
  assign dataArray_io_multiWrite_1_enable = dataArray_io_multiWrite_1_enable_REG; // @[ReservationStation.scala 633:14]
  assign dataArray_io_multiWrite_1_addr_0 = _dataArray_io_multiWrite_1_addr_0_T_8 | allocateDataCapture_3; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_1_addr_1 = _dataArray_io_multiWrite_1_addr_1_T_8 | allocateDataCapture_4; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_1_addr_2 = _dataArray_io_multiWrite_1_addr_2_T_8 | allocateDataCapture_5; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_1_data = dataArray_io_multiWrite_1_data_r; // @[ReservationStation.scala 639:12]
  assign dataArray_io_multiWrite_2_enable = dataArray_io_multiWrite_2_enable_REG; // @[ReservationStation.scala 633:14]
  assign dataArray_io_multiWrite_2_addr_0 = _dataArray_io_multiWrite_2_addr_0_T_8 | allocateDataCapture_6; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_2_addr_1 = _dataArray_io_multiWrite_2_addr_1_T_8 | allocateDataCapture_7; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_2_addr_2 = _dataArray_io_multiWrite_2_addr_2_T_8 | allocateDataCapture_8; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_2_data = dataArray_io_multiWrite_2_data_r; // @[ReservationStation.scala 639:12]
  assign dataArray_io_multiWrite_3_enable = dataArray_io_multiWrite_3_enable_REG; // @[ReservationStation.scala 633:14]
  assign dataArray_io_multiWrite_3_addr_0 = _dataArray_io_multiWrite_3_addr_0_T_8 | allocateDataCapture_9; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_3_addr_1 = _dataArray_io_multiWrite_3_addr_1_T_8 | allocateDataCapture_10; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_3_addr_2 = _dataArray_io_multiWrite_3_addr_2_T_8 | allocateDataCapture_11; // @[ReservationStation.scala 637:68]
  assign dataArray_io_multiWrite_3_data = dataArray_io_multiWrite_3_data_r; // @[ReservationStation.scala 639:12]
  assign dataArray_io_partialWrite_0_enable = cond11_0 ? _GEN_450 : midFinished2T2_0; // @[ReservationStation.scala 844:18 818:43]
  assign dataArray_io_partialWrite_0_addr = midIssuePtrOHT2_0; // @[ReservationStation.scala 820:41]
  assign dataArray_io_partialWrite_0_data_0 = writeData[63:0]; // @[ReservationStation.scala 824:56]
  assign dataArray_io_partialWrite_0_data_1 = {{7'd0}, writeData[120:64]}; // @[ReservationStation.scala 825:44]
  assign payloadArray_clock = clock;
  assign payloadArray_io_read_0_addr = select_io_grant_0_bits; // @[ReservationStation.scala 368:34]
  assign payloadArray_io_read_1_addr = s1_oldestSel_age_io_out; // @[SelectPolicy.scala 177:19 179:14]
  assign payloadArray_io_write_0_enable = s1_dispatchUops_dup_1_0_valid; // @[ReservationStation.scala 471:25]
  assign payloadArray_io_write_0_addr = s1_allocatePtrOH_dup_1_0; // @[ReservationStation.scala 472:23]
  assign payloadArray_io_write_0_data_cf_pd_isRVC = s1_dispatchUops_dup_1_0_bits_cf_pd_isRVC; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_cf_pd_brType = s1_dispatchUops_dup_1_0_bits_cf_pd_brType; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_cf_pd_isCall = s1_dispatchUops_dup_1_0_bits_cf_pd_isCall; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_cf_pd_isRet = s1_dispatchUops_dup_1_0_bits_cf_pd_isRet; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_cf_pred_taken = s1_dispatchUops_dup_1_0_bits_cf_pred_taken; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_cf_ftqPtr_flag = s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_flag; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_cf_ftqPtr_value = s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_value; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_cf_ftqOffset = s1_dispatchUops_dup_1_0_bits_cf_ftqOffset; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fuType = s1_dispatchUops_dup_1_0_bits_ctrl_fuType; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fuOpType = s1_dispatchUops_dup_1_0_bits_ctrl_fuOpType; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_rfWen = s1_dispatchUops_dup_1_0_bits_ctrl_rfWen; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpWen = s1_dispatchUops_dup_1_0_bits_ctrl_fpWen; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_imm = s1_dispatchUops_dup_1_0_bits_ctrl_imm; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_isAddSub = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_isAddSub; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_typeTagIn = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_typeTagOut = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagOut; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_fromInt = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fromInt; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_wflags = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_wflags; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_fpWen = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fpWen; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_fmaCmd = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_div = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_div; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_sqrt = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_sqrt; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_fcvt = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fcvt; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_typ = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typ; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_fmt = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmt; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_ren3 = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_ren3; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_ctrl_fpu_rm = s1_dispatchUops_dup_1_0_bits_ctrl_fpu_rm; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_pdest = s1_dispatchUops_dup_1_0_bits_pdest; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_robIdx_flag = s1_dispatchUops_dup_1_0_bits_robIdx_flag; // @[ReservationStation.scala 473:23]
  assign payloadArray_io_write_0_data_robIdx_value = s1_dispatchUops_dup_1_0_bits_robIdx_value; // @[ReservationStation.scala 473:23]
  assign s1_oldestSel_age_clock = clock;
  assign s1_oldestSel_age_reset = reset;
  assign s1_oldestSel_age_io_enq_0 = enqVec_REG; // @[ReservationStation.scala 361:{23,23}]
  assign s1_oldestSel_age_io_deq = statusArray_io_flushed; // @[SelectPolicy.scala 176:16]
  assign oldestSelection_io_oldest_valid = |_s1_oldestSel_out_valid_T; // @[SelectPolicy.scala 178:42]
  assign immExt_io_data_in_0 = io_srcRegValue_0_0; // @[DataArray.scala 163:23]
  assign immExt_io_data_in_1 = io_srcRegValue_0_1; // @[DataArray.scala 163:23]
  assign immExt_io_data_in_2 = io_srcRegValue_0_2; // @[DataArray.scala 163:23]
  assign dataSelect_io_doOverride_0 = oldestSelection_io_isOverrided_0; // @[ReservationStation.scala 402:29 504:21]
  assign dataSelect_io_readData_0_0 = dataArray_io_read_0_data_0; // @[ReservationStation.scala 693:26]
  assign dataSelect_io_readData_0_1 = dataArray_io_read_0_data_1; // @[ReservationStation.scala 693:26]
  assign dataSelect_io_readData_0_2 = dataArray_io_read_0_data_2; // @[ReservationStation.scala 693:26]
  assign dataSelect_io_readData_1_0 = dataArray_io_read_1_data_0; // @[ReservationStation.scala 693:26]
  assign dataSelect_io_readData_1_1 = dataArray_io_read_1_data_1; // @[ReservationStation.scala 693:26]
  assign dataSelect_io_readData_1_2 = dataArray_io_read_1_data_2; // @[ReservationStation.scala 693:26]
  assign dataSelect_io_fromSlowPorts_0_0 = {dataSelect_io_fromSlowPorts_0_0_hi,dataSelect_io_fromSlowPorts_0_0_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_0_1 = {dataSelect_io_fromSlowPorts_0_1_hi,dataSelect_io_fromSlowPorts_0_1_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_0_2 = {dataSelect_io_fromSlowPorts_0_2_hi,dataSelect_io_fromSlowPorts_0_2_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_1_0 = {dataSelect_io_fromSlowPorts_1_0_hi,dataSelect_io_fromSlowPorts_1_0_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_1_1 = {dataSelect_io_fromSlowPorts_1_1_hi,dataSelect_io_fromSlowPorts_1_1_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_1_2 = {dataSelect_io_fromSlowPorts_1_2_hi,dataSelect_io_fromSlowPorts_1_2_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_2_0 = {dataSelect_io_fromSlowPorts_2_0_hi,dataSelect_io_fromSlowPorts_2_0_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_2_1 = {dataSelect_io_fromSlowPorts_2_1_hi,dataSelect_io_fromSlowPorts_2_1_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_fromSlowPorts_2_2 = {dataSelect_io_fromSlowPorts_2_2_hi,dataSelect_io_fromSlowPorts_2_2_lo}; // @[ReservationStation.scala 697:103]
  assign dataSelect_io_slowData_0 = dataArray_io_multiWrite_0_data; // @[ReservationStation.scala 700:26]
  assign dataSelect_io_slowData_1 = dataArray_io_multiWrite_1_data; // @[ReservationStation.scala 700:26]
  assign dataSelect_io_slowData_2 = dataArray_io_multiWrite_2_data; // @[ReservationStation.scala 700:26]
  assign dataSelect_io_slowData_3 = dataArray_io_multiWrite_3_data; // @[ReservationStation.scala 700:26]
  assign dataSelect_io_enqBypass_0_0 = canBypass & ~s1_issue_oldest_0 & ~select_io_grant_0_valid; // @[ReservationStation.scala 512:62]
  assign dataSelect_io_enqData_0_0_bits = immExt_io_data_out_0; // @[ReservationStation.scala 589:29 593:12]
  assign dataSelect_io_enqData_0_1_bits = immExt_io_data_out_1; // @[ReservationStation.scala 589:29 593:12]
  assign dataSelect_io_enqData_0_2_bits = immExt_io_data_out_2; // @[ReservationStation.scala 589:29 593:12]
  assign bypassNetwork_clock = clock;
  assign bypassNetwork_reset = reset;
  assign bypassNetwork_io_hold = ~s2_deq_0_ready | ~s1_out_0_valid; // @[ReservationStation.scala 766:49]
  assign bypassNetwork_io_source_0 = cond2_0 ? dataArray_io_partialWrite_0_data_0 : dataSelect_io_deqData_0_0; // @[ReservationStation.scala 874:18 710:29 875:33]
  assign bypassNetwork_io_source_1 = cond2_0 ? dataArray_io_partialWrite_0_data_1 : dataSelect_io_deqData_0_1; // @[ReservationStation.scala 874:18 710:29 876:33]
  assign bypassNetwork_io_source_2 = dataSelect_io_deqData_0_2; // @[ReservationStation.scala 404:20 710:29]
  assign bypassNetwork_io_bypass_0_valid_0 = _s1_issuePtrOH_0_valid_T ? normalFastWakeupMatch_0_0 : s1_fastWakeup_0_0_0; // @[ReservationStation.scala 761:40]
  assign bypassNetwork_io_bypass_0_valid_1 = _s1_issuePtrOH_0_valid_T ? normalFastWakeupMatch_1_0 : s1_fastWakeup_0_1_0; // @[ReservationStation.scala 761:40]
  assign bypassNetwork_io_bypass_0_valid_2 = _s1_issuePtrOH_0_valid_T ? normalFastWakeupMatch_2_0 : s1_fastWakeup_0_2_0; // @[ReservationStation.scala 761:40]
  assign bypassNetwork_io_bypass_0_data = io_fastDatas_0; // @[ReservationStation.scala 770:17]
  always @(posedge clock) begin
    emptyThisCycle <= numEmptyAfterS1 + _GEN_479; // @[ReservationStation.scala 319:39]
    allocateThisCycle <= {{1'd0}, s0_doEnqueue_0}; // @[ReservationStation.scala 323:42]
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 361:90]
      enqVec_REG <= s0_allocatePtrOH_0;
    end else begin
      enqVec_REG <= 8'h0;
    end
    s1_dispatchUops_dup_0_0_valid <= _s0_doEnqueue_0_T & _s0_doEnqueue_0_T_1; // @[ReservationStation.scala 416:28]
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_srcType_0 <= io_fromDispatch_0_bits_ctrl_srcType_0; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_srcType_1 <= io_fromDispatch_0_bits_ctrl_srcType_1; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_srcType_2 <= io_fromDispatch_0_bits_ctrl_srcType_2; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_rfWen <= io_fromDispatch_0_bits_ctrl_rfWen; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_fpWen <= io_fromDispatch_0_bits_ctrl_fpWen; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_fpu_isAddSub <= io_fromDispatch_0_bits_ctrl_fpu_isAddSub; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_fpu_typeTagIn <= io_fromDispatch_0_bits_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_fpu_fmaCmd <= io_fromDispatch_0_bits_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_fpu_ren3 <= io_fromDispatch_0_bits_ctrl_fpu_ren3; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_ctrl_fpu_rm <= io_fromDispatch_0_bits_ctrl_fpu_rm; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_srcState_0 <= io_fromDispatch_0_bits_srcState_0; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_srcState_1 <= io_fromDispatch_0_bits_srcState_1; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_srcState_2 <= io_fromDispatch_0_bits_srcState_2; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_psrc_0 <= io_fromDispatch_0_bits_psrc_0; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_psrc_1 <= io_fromDispatch_0_bits_psrc_1; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_psrc_2 <= io_fromDispatch_0_bits_psrc_2; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_pdest <= io_fromDispatch_0_bits_pdest; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_robIdx_flag <= io_fromDispatch_0_bits_robIdx_flag; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_0_0_bits_robIdx_value <= io_fromDispatch_0_bits_robIdx_value; // @[ReservationStation.scala 419:16]
    end
    s1_dispatchUops_dup_1_0_valid <= _s0_doEnqueue_0_T & _s0_doEnqueue_0_T_1; // @[ReservationStation.scala 416:28]
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_pd_isRVC <= io_fromDispatch_0_bits_cf_pd_isRVC; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_pd_brType <= io_fromDispatch_0_bits_cf_pd_brType; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_pd_isCall <= io_fromDispatch_0_bits_cf_pd_isCall; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_pd_isRet <= io_fromDispatch_0_bits_cf_pd_isRet; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_pred_taken <= io_fromDispatch_0_bits_cf_pred_taken; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_flag <= io_fromDispatch_0_bits_cf_ftqPtr_flag; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_value <= io_fromDispatch_0_bits_cf_ftqPtr_value; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_cf_ftqOffset <= io_fromDispatch_0_bits_cf_ftqOffset; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fuType <= io_fromDispatch_0_bits_ctrl_fuType; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fuOpType <= io_fromDispatch_0_bits_ctrl_fuOpType; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_rfWen <= io_fromDispatch_0_bits_ctrl_rfWen; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpWen <= io_fromDispatch_0_bits_ctrl_fpWen; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_imm <= io_fromDispatch_0_bits_ctrl_imm; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_isAddSub <= io_fromDispatch_0_bits_ctrl_fpu_isAddSub; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagIn <= io_fromDispatch_0_bits_ctrl_fpu_typeTagIn; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagOut <= io_fromDispatch_0_bits_ctrl_fpu_typeTagOut; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fromInt <= io_fromDispatch_0_bits_ctrl_fpu_fromInt; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_wflags <= io_fromDispatch_0_bits_ctrl_fpu_wflags; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fpWen <= io_fromDispatch_0_bits_ctrl_fpu_fpWen; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmaCmd <= io_fromDispatch_0_bits_ctrl_fpu_fmaCmd; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_div <= io_fromDispatch_0_bits_ctrl_fpu_div; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_sqrt <= io_fromDispatch_0_bits_ctrl_fpu_sqrt; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fcvt <= io_fromDispatch_0_bits_ctrl_fpu_fcvt; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typ <= io_fromDispatch_0_bits_ctrl_fpu_typ; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmt <= io_fromDispatch_0_bits_ctrl_fpu_fmt; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_ren3 <= io_fromDispatch_0_bits_ctrl_fpu_ren3; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_ctrl_fpu_rm <= io_fromDispatch_0_bits_ctrl_fpu_rm; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_pdest <= io_fromDispatch_0_bits_pdest; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_robIdx_flag <= io_fromDispatch_0_bits_robIdx_flag; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_1_0_bits_robIdx_value <= io_fromDispatch_0_bits_robIdx_value; // @[ReservationStation.scala 419:16]
    end
    s1_dispatchUops_dup_2_0_valid <= _s0_doEnqueue_0_T & _s0_doEnqueue_0_T_1; // @[ReservationStation.scala 416:28]
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_2_0_bits_ctrl_srcType_0 <= io_fromDispatch_0_bits_ctrl_srcType_0; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_2_0_bits_ctrl_srcType_1 <= io_fromDispatch_0_bits_ctrl_srcType_1; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_2_0_bits_ctrl_srcType_2 <= io_fromDispatch_0_bits_ctrl_srcType_2; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_2_0_bits_srcState_0 <= io_fromDispatch_0_bits_srcState_0; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_2_0_bits_srcState_1 <= io_fromDispatch_0_bits_srcState_1; // @[ReservationStation.scala 419:16]
    end
    if (s0_doEnqueue_0) begin // @[ReservationStation.scala 418:21]
      s1_dispatchUops_dup_2_0_bits_srcState_2 <= io_fromDispatch_0_bits_srcState_2; // @[ReservationStation.scala 419:16]
    end
    s1_allocatePtrOH_dup_0_0 <= select_io_allocate_0_bits; // @[ReservationStation.scala 273:{33,33}]
    s1_allocatePtrOH_dup_1_0 <= select_io_allocate_0_bits; // @[ReservationStation.scala 273:{33,33}]
    s1_allocatePtrOH_dup_2_0 <= select_io_allocate_0_bits; // @[ReservationStation.scala 273:{33,33}]
    s1_enqWakeup_0_0 <= {s0_enqWakeup_0_0_hi,s0_enqWakeup_0_0_lo}; // @[ReservationStation.scala 341:100]
    s1_enqWakeup_0_1 <= {s0_enqWakeup_0_1_hi,s0_enqWakeup_0_1_lo}; // @[ReservationStation.scala 341:100]
    s1_enqWakeup_0_2 <= {s0_enqWakeup_0_2_hi,s0_enqWakeup_0_2_lo}; // @[ReservationStation.scala 341:100]
    s1_enqDataCapture_0_0 <= {s0_enqDataCapture_0_0_hi,s0_enqDataCapture_0_0_lo}; // @[ReservationStation.scala 342:104]
    s1_enqDataCapture_0_1 <= {s0_enqDataCapture_0_1_hi,s0_enqDataCapture_0_1_lo}; // @[ReservationStation.scala 342:104]
    s1_enqDataCapture_0_2 <= {s0_enqDataCapture_0_2_hi,s0_enqDataCapture_0_2_lo}; // @[ReservationStation.scala 342:104]
    s1_fastWakeup_0_0_0 <= io_fastUopsIn_0_valid & dataCond_12; // @[ReservationStation.scala 344:83]
    s1_fastWakeup_0_1_0 <= io_fastUopsIn_0_valid & dataCond_13; // @[ReservationStation.scala 344:83]
    s1_fastWakeup_0_2_0 <= io_fastUopsIn_0_valid & dataCond_14; // @[ReservationStation.scala 344:83]
    slowWakeupMatchVec_0_0 <= statusArray_io_wakeupMatch_0_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_0_1 <= statusArray_io_wakeupMatch_0_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_0_2 <= statusArray_io_wakeupMatch_0_2[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_1_0 <= statusArray_io_wakeupMatch_1_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_1_1 <= statusArray_io_wakeupMatch_1_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_1_2 <= statusArray_io_wakeupMatch_1_2[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_2_0 <= statusArray_io_wakeupMatch_2_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_2_1 <= statusArray_io_wakeupMatch_2_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_2_2 <= statusArray_io_wakeupMatch_2_2[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_3_0 <= statusArray_io_wakeupMatch_3_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_3_1 <= statusArray_io_wakeupMatch_3_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_3_2 <= statusArray_io_wakeupMatch_3_2[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_4_0 <= statusArray_io_wakeupMatch_4_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_4_1 <= statusArray_io_wakeupMatch_4_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_4_2 <= statusArray_io_wakeupMatch_4_2[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_5_0 <= statusArray_io_wakeupMatch_5_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_5_1 <= statusArray_io_wakeupMatch_5_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_5_2 <= statusArray_io_wakeupMatch_5_2[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_6_0 <= statusArray_io_wakeupMatch_6_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_6_1 <= statusArray_io_wakeupMatch_6_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_6_2 <= statusArray_io_wakeupMatch_6_2[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_7_0 <= statusArray_io_wakeupMatch_7_0[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_7_1 <= statusArray_io_wakeupMatch_7_1[4:1]; // @[ReservationStation.scala 629:67]
    slowWakeupMatchVec_7_2 <= statusArray_io_wakeupMatch_7_2[4:1]; // @[ReservationStation.scala 629:67]
    dataArray_io_multiWrite_0_enable_REG <= io_slowPorts_0_valid; // @[ReservationStation.scala 633:24]
    if (io_slowPorts_0_valid) begin // @[Reg.scala 17:18]
      dataArray_io_multiWrite_0_data_r <= io_slowPorts_0_bits_data; // @[Reg.scala 17:22]
    end
    dataArray_io_multiWrite_1_enable_REG <= io_slowPorts_1_valid; // @[ReservationStation.scala 633:24]
    if (io_slowPorts_1_valid) begin // @[Reg.scala 17:18]
      dataArray_io_multiWrite_1_data_r <= io_slowPorts_1_bits_data; // @[Reg.scala 17:22]
    end
    dataArray_io_multiWrite_2_enable_REG <= io_slowPorts_2_valid; // @[ReservationStation.scala 633:24]
    if (io_slowPorts_2_valid) begin // @[Reg.scala 17:18]
      dataArray_io_multiWrite_2_data_r <= io_slowPorts_2_bits_data; // @[Reg.scala 17:22]
    end
    dataArray_io_multiWrite_3_enable_REG <= io_slowPorts_3_valid; // @[ReservationStation.scala 633:24]
    if (io_slowPorts_3_valid) begin // @[Reg.scala 17:18]
      dataArray_io_multiWrite_3_data_r <= io_slowPorts_3_bits_data; // @[Reg.scala 17:22]
    end
    fastWakeupMatch_0_0_0 <= statusArray_io_wakeupMatch_0_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_0_1_0 <= statusArray_io_wakeupMatch_0_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_0_2_0 <= statusArray_io_wakeupMatch_0_2[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_1_0_0 <= statusArray_io_wakeupMatch_1_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_1_1_0 <= statusArray_io_wakeupMatch_1_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_1_2_0 <= statusArray_io_wakeupMatch_1_2[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_2_0_0 <= statusArray_io_wakeupMatch_2_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_2_1_0 <= statusArray_io_wakeupMatch_2_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_2_2_0 <= statusArray_io_wakeupMatch_2_2[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_3_0_0 <= statusArray_io_wakeupMatch_3_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_3_1_0 <= statusArray_io_wakeupMatch_3_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_3_2_0 <= statusArray_io_wakeupMatch_3_2[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_4_0_0 <= statusArray_io_wakeupMatch_4_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_4_1_0 <= statusArray_io_wakeupMatch_4_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_4_2_0 <= statusArray_io_wakeupMatch_4_2[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_5_0_0 <= statusArray_io_wakeupMatch_5_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_5_1_0 <= statusArray_io_wakeupMatch_5_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_5_2_0 <= statusArray_io_wakeupMatch_5_2[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_6_0_0 <= statusArray_io_wakeupMatch_6_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_6_1_0 <= statusArray_io_wakeupMatch_6_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_6_2_0 <= statusArray_io_wakeupMatch_6_2[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_7_0_0 <= statusArray_io_wakeupMatch_7_0[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_7_1_0 <= statusArray_io_wakeupMatch_7_1[0]; // @[ReservationStation.scala 721:65]
    fastWakeupMatch_7_2_0 <= statusArray_io_wakeupMatch_7_2[0]; // @[ReservationStation.scala 721:65]
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      s2_issuePtr_0 <= s1_issuePtr_0; // @[Reg.scala 17:22]
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 515:33]
        s2_issuePtrOH_0 <= s1_in_oldestPtrOH_bits;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 516:10]
        s2_issuePtrOH_0 <= select_io_grant_0_bits;
      end else begin
        s2_issuePtrOH_0 <= s1_allocatePtrOH_dup_0_0;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 520:32]
        s2_first_issue_0 <= statusArray_io_isFirstIssue_2;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 521:10]
        s2_first_issue_0 <= statusArray_io_isFirstIssue_1;
      end else begin
        s2_first_issue_0 <= statusArray_io_update_0_data_isFirstIssue;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 523:31]
        s2_all_src_ready_0 <= statusArray_io_allSrcReady_2;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 524:12]
        s2_all_src_ready_0 <= statusArray_io_allSrcReady_1;
      end else begin
        s2_all_src_ready_0 <= _s1_all_src_ready_0_T_2;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_robIdx_flag <= payloadArray_io_read_1_data_robIdx_flag;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_robIdx_flag <= payloadArray_io_read_0_data_robIdx_flag;
      end else begin
        data_uop_robIdx_flag <= s1_dispatchUops_dup_0_0_bits_robIdx_flag;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_robIdx_value <= payloadArray_io_read_1_data_robIdx_value;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_robIdx_value <= payloadArray_io_read_0_data_robIdx_value;
      end else begin
        data_uop_robIdx_value <= s1_dispatchUops_dup_0_0_bits_robIdx_value;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_ctrl_rfWen <= payloadArray_io_read_1_data_ctrl_rfWen;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_ctrl_rfWen <= payloadArray_io_read_0_data_ctrl_rfWen;
      end else begin
        data_uop_ctrl_rfWen <= s1_dispatchUops_dup_0_0_bits_ctrl_rfWen;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_ctrl_fpWen <= payloadArray_io_read_1_data_ctrl_fpWen;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_ctrl_fpWen <= payloadArray_io_read_0_data_ctrl_fpWen;
      end else begin
        data_uop_ctrl_fpWen <= s1_dispatchUops_dup_0_0_bits_ctrl_fpWen;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_ctrl_fpu_isAddSub <= payloadArray_io_read_1_data_ctrl_fpu_isAddSub;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_ctrl_fpu_isAddSub <= payloadArray_io_read_0_data_ctrl_fpu_isAddSub;
      end else begin
        data_uop_ctrl_fpu_isAddSub <= s1_dispatchUops_dup_0_0_bits_ctrl_fpu_isAddSub;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_ctrl_fpu_typeTagIn <= payloadArray_io_read_1_data_ctrl_fpu_typeTagIn;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_ctrl_fpu_typeTagIn <= payloadArray_io_read_0_data_ctrl_fpu_typeTagIn;
      end else begin
        data_uop_ctrl_fpu_typeTagIn <= s1_dispatchUops_dup_0_0_bits_ctrl_fpu_typeTagIn;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_ctrl_fpu_fmaCmd <= payloadArray_io_read_1_data_ctrl_fpu_fmaCmd;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_ctrl_fpu_fmaCmd <= payloadArray_io_read_0_data_ctrl_fpu_fmaCmd;
      end else begin
        data_uop_ctrl_fpu_fmaCmd <= s1_dispatchUops_dup_0_0_bits_ctrl_fpu_fmaCmd;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_ctrl_fpu_ren3 <= payloadArray_io_read_1_data_ctrl_fpu_ren3;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_ctrl_fpu_ren3 <= payloadArray_io_read_0_data_ctrl_fpu_ren3;
      end else begin
        data_uop_ctrl_fpu_ren3 <= s1_dispatchUops_dup_0_0_bits_ctrl_fpu_ren3;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_ctrl_fpu_rm <= payloadArray_io_read_1_data_ctrl_fpu_rm;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_ctrl_fpu_rm <= payloadArray_io_read_0_data_ctrl_fpu_rm;
      end else begin
        data_uop_ctrl_fpu_rm <= s1_dispatchUops_dup_0_0_bits_ctrl_fpu_rm;
      end
    end
    if (s1_out_fire_0) begin // @[Reg.scala 17:18]
      if (s1_issue_oldest_0) begin // @[ReservationStation.scala 518:30]
        data_uop_pdest <= payloadArray_io_read_1_data_pdest;
      end else if (select_io_grant_0_valid) begin // @[ReservationStation.scala 519:10]
        data_uop_pdest <= payloadArray_io_read_0_data_pdest;
      end else begin
        data_uop_pdest <= s1_dispatchUops_dup_0_0_bits_pdest;
      end
    end
    midFinished2T2_0 <= midFinished2T1_0; // @[ReservationStation.scala 816:57]
    if (midFinished2T1_0) begin // @[Reg.scala 17:18]
      midIssuePtrT2_0 <= midIssuePtrT1_0; // @[Reg.scala 17:22]
    end
    if (midFinished2T0_0) begin // @[Reg.scala 17:18]
      midIssuePtrOHT1_0 <= s2_issuePtrOH_0; // @[Reg.scala 17:22]
    end
    if (midFinished2T0_0) begin // @[Reg.scala 17:18]
      midIssuePtrT1_0 <= s2_issuePtr_0; // @[Reg.scala 17:22]
    end
    midFinished2T1_0 <= midFinished2_0 & ~_T_142; // @[ReservationStation.scala 807:9]
    if (midFinished2T1_0) begin // @[Reg.scala 17:18]
      midIssuePtrOHT2_0 <= midIssuePtrOHT1_0; // @[Reg.scala 17:22]
    end
    if (hasCond12) begin // @[Reg.scala 17:18]
      io_fmaMid_0_in_bits_rfp_prod_sign <= io_fmaMid_0_out_bits_fp_prod_sign; // @[Reg.scala 17:22]
    end
    if (hasCond12) begin // @[Reg.scala 17:18]
      io_fmaMid_0_in_bits_rfp_prod_exp <= io_fmaMid_0_out_bits_fp_prod_exp; // @[Reg.scala 17:22]
    end
    if (hasCond12) begin // @[Reg.scala 17:18]
      io_fmaMid_0_in_bits_rfp_prod_sig <= io_fmaMid_0_out_bits_fp_prod_sig; // @[Reg.scala 17:22]
    end
    if (hasCond12) begin // @[Reg.scala 17:18]
      io_fmaMid_0_in_bits_rinter_flags_isNaN <= io_fmaMid_0_out_bits_inter_flags_isNaN; // @[Reg.scala 17:22]
    end
    if (hasCond12) begin // @[Reg.scala 17:18]
      io_fmaMid_0_in_bits_rinter_flags_isInf <= io_fmaMid_0_out_bits_inter_flags_isInf; // @[Reg.scala 17:22]
    end
    if (hasCond12) begin // @[Reg.scala 17:18]
      io_fmaMid_0_in_bits_rinter_flags_isInv <= io_fmaMid_0_out_bits_inter_flags_isInv; // @[Reg.scala 17:22]
    end
    io_perf_0_value_REG <= &statusArray_io_isValid; // @[ReservationStation.scala 966:56]
    io_perf_0_value_REG_1 <= io_perf_0_value_REG; // @[PerfCounterUtils.scala 188:27]
  end
  always @(posedge clock or posedge reset) begin
    if (reset) begin // @[ReservationStation.scala 284:52]
      validAfterAllocate <= 8'h0;
    end else begin
      validAfterAllocate <= statusArray_io_isValidNext | validUpdateByAllocate;
    end
  end
  always @(posedge clock or posedge reset) begin
    if (reset) begin // @[PipelineConnect.scala 111:21]
      valid <= 1'h0; // @[PipelineConnect.scala 111:29]
    end else begin
      valid <= s1_out_fire_0 | _GEN_332;
    end
  end
  always @(posedge clock or posedge reset) begin
    if (reset) begin // @[ReservationStation.scala 860:24]
      hasCond12Reg <= 1'h0; // @[ReservationStation.scala 861:22]
    end else begin
      hasCond12Reg <= hasCond12 | _GEN_459;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_REG_INIT
  _RAND_0 = {1{`RANDOM}};
  validAfterAllocate = _RAND_0[7:0];
  _RAND_1 = {1{`RANDOM}};
  emptyThisCycle = _RAND_1[2:0];
  _RAND_2 = {1{`RANDOM}};
  allocateThisCycle = _RAND_2[1:0];
  _RAND_3 = {1{`RANDOM}};
  enqVec_REG = _RAND_3[7:0];
  _RAND_4 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_valid = _RAND_4[0:0];
  _RAND_5 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_srcType_0 = _RAND_5[1:0];
  _RAND_6 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_srcType_1 = _RAND_6[1:0];
  _RAND_7 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_srcType_2 = _RAND_7[1:0];
  _RAND_8 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_rfWen = _RAND_8[0:0];
  _RAND_9 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_fpWen = _RAND_9[0:0];
  _RAND_10 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_isAddSub = _RAND_10[0:0];
  _RAND_11 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_typeTagIn = _RAND_11[0:0];
  _RAND_12 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_fmaCmd = _RAND_12[1:0];
  _RAND_13 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_ren3 = _RAND_13[0:0];
  _RAND_14 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_ctrl_fpu_rm = _RAND_14[2:0];
  _RAND_15 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_srcState_0 = _RAND_15[0:0];
  _RAND_16 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_srcState_1 = _RAND_16[0:0];
  _RAND_17 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_srcState_2 = _RAND_17[0:0];
  _RAND_18 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_psrc_0 = _RAND_18[5:0];
  _RAND_19 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_psrc_1 = _RAND_19[5:0];
  _RAND_20 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_psrc_2 = _RAND_20[5:0];
  _RAND_21 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_pdest = _RAND_21[5:0];
  _RAND_22 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_robIdx_flag = _RAND_22[0:0];
  _RAND_23 = {1{`RANDOM}};
  s1_dispatchUops_dup_0_0_bits_robIdx_value = _RAND_23[4:0];
  _RAND_24 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_valid = _RAND_24[0:0];
  _RAND_25 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_pd_isRVC = _RAND_25[0:0];
  _RAND_26 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_pd_brType = _RAND_26[1:0];
  _RAND_27 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_pd_isCall = _RAND_27[0:0];
  _RAND_28 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_pd_isRet = _RAND_28[0:0];
  _RAND_29 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_pred_taken = _RAND_29[0:0];
  _RAND_30 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_flag = _RAND_30[0:0];
  _RAND_31 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_ftqPtr_value = _RAND_31[2:0];
  _RAND_32 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_cf_ftqOffset = _RAND_32[2:0];
  _RAND_33 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fuType = _RAND_33[3:0];
  _RAND_34 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fuOpType = _RAND_34[6:0];
  _RAND_35 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_rfWen = _RAND_35[0:0];
  _RAND_36 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpWen = _RAND_36[0:0];
  _RAND_37 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_imm = _RAND_37[19:0];
  _RAND_38 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_isAddSub = _RAND_38[0:0];
  _RAND_39 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagIn = _RAND_39[0:0];
  _RAND_40 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typeTagOut = _RAND_40[0:0];
  _RAND_41 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fromInt = _RAND_41[0:0];
  _RAND_42 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_wflags = _RAND_42[0:0];
  _RAND_43 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fpWen = _RAND_43[0:0];
  _RAND_44 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmaCmd = _RAND_44[1:0];
  _RAND_45 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_div = _RAND_45[0:0];
  _RAND_46 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_sqrt = _RAND_46[0:0];
  _RAND_47 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fcvt = _RAND_47[0:0];
  _RAND_48 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_typ = _RAND_48[1:0];
  _RAND_49 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_fmt = _RAND_49[1:0];
  _RAND_50 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_ren3 = _RAND_50[0:0];
  _RAND_51 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_ctrl_fpu_rm = _RAND_51[2:0];
  _RAND_52 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_pdest = _RAND_52[5:0];
  _RAND_53 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_robIdx_flag = _RAND_53[0:0];
  _RAND_54 = {1{`RANDOM}};
  s1_dispatchUops_dup_1_0_bits_robIdx_value = _RAND_54[4:0];
  _RAND_55 = {1{`RANDOM}};
  s1_dispatchUops_dup_2_0_valid = _RAND_55[0:0];
  _RAND_56 = {1{`RANDOM}};
  s1_dispatchUops_dup_2_0_bits_ctrl_srcType_0 = _RAND_56[1:0];
  _RAND_57 = {1{`RANDOM}};
  s1_dispatchUops_dup_2_0_bits_ctrl_srcType_1 = _RAND_57[1:0];
  _RAND_58 = {1{`RANDOM}};
  s1_dispatchUops_dup_2_0_bits_ctrl_srcType_2 = _RAND_58[1:0];
  _RAND_59 = {1{`RANDOM}};
  s1_dispatchUops_dup_2_0_bits_srcState_0 = _RAND_59[0:0];
  _RAND_60 = {1{`RANDOM}};
  s1_dispatchUops_dup_2_0_bits_srcState_1 = _RAND_60[0:0];
  _RAND_61 = {1{`RANDOM}};
  s1_dispatchUops_dup_2_0_bits_srcState_2 = _RAND_61[0:0];
  _RAND_62 = {1{`RANDOM}};
  s1_allocatePtrOH_dup_0_0 = _RAND_62[7:0];
  _RAND_63 = {1{`RANDOM}};
  s1_allocatePtrOH_dup_1_0 = _RAND_63[7:0];
  _RAND_64 = {1{`RANDOM}};
  s1_allocatePtrOH_dup_2_0 = _RAND_64[7:0];
  _RAND_65 = {1{`RANDOM}};
  s1_enqWakeup_0_0 = _RAND_65[3:0];
  _RAND_66 = {1{`RANDOM}};
  s1_enqWakeup_0_1 = _RAND_66[3:0];
  _RAND_67 = {1{`RANDOM}};
  s1_enqWakeup_0_2 = _RAND_67[3:0];
  _RAND_68 = {1{`RANDOM}};
  s1_enqDataCapture_0_0 = _RAND_68[3:0];
  _RAND_69 = {1{`RANDOM}};
  s1_enqDataCapture_0_1 = _RAND_69[3:0];
  _RAND_70 = {1{`RANDOM}};
  s1_enqDataCapture_0_2 = _RAND_70[3:0];
  _RAND_71 = {1{`RANDOM}};
  s1_fastWakeup_0_0_0 = _RAND_71[0:0];
  _RAND_72 = {1{`RANDOM}};
  s1_fastWakeup_0_1_0 = _RAND_72[0:0];
  _RAND_73 = {1{`RANDOM}};
  s1_fastWakeup_0_2_0 = _RAND_73[0:0];
  _RAND_74 = {1{`RANDOM}};
  valid = _RAND_74[0:0];
  _RAND_75 = {1{`RANDOM}};
  slowWakeupMatchVec_0_0 = _RAND_75[3:0];
  _RAND_76 = {1{`RANDOM}};
  slowWakeupMatchVec_0_1 = _RAND_76[3:0];
  _RAND_77 = {1{`RANDOM}};
  slowWakeupMatchVec_0_2 = _RAND_77[3:0];
  _RAND_78 = {1{`RANDOM}};
  slowWakeupMatchVec_1_0 = _RAND_78[3:0];
  _RAND_79 = {1{`RANDOM}};
  slowWakeupMatchVec_1_1 = _RAND_79[3:0];
  _RAND_80 = {1{`RANDOM}};
  slowWakeupMatchVec_1_2 = _RAND_80[3:0];
  _RAND_81 = {1{`RANDOM}};
  slowWakeupMatchVec_2_0 = _RAND_81[3:0];
  _RAND_82 = {1{`RANDOM}};
  slowWakeupMatchVec_2_1 = _RAND_82[3:0];
  _RAND_83 = {1{`RANDOM}};
  slowWakeupMatchVec_2_2 = _RAND_83[3:0];
  _RAND_84 = {1{`RANDOM}};
  slowWakeupMatchVec_3_0 = _RAND_84[3:0];
  _RAND_85 = {1{`RANDOM}};
  slowWakeupMatchVec_3_1 = _RAND_85[3:0];
  _RAND_86 = {1{`RANDOM}};
  slowWakeupMatchVec_3_2 = _RAND_86[3:0];
  _RAND_87 = {1{`RANDOM}};
  slowWakeupMatchVec_4_0 = _RAND_87[3:0];
  _RAND_88 = {1{`RANDOM}};
  slowWakeupMatchVec_4_1 = _RAND_88[3:0];
  _RAND_89 = {1{`RANDOM}};
  slowWakeupMatchVec_4_2 = _RAND_89[3:0];
  _RAND_90 = {1{`RANDOM}};
  slowWakeupMatchVec_5_0 = _RAND_90[3:0];
  _RAND_91 = {1{`RANDOM}};
  slowWakeupMatchVec_5_1 = _RAND_91[3:0];
  _RAND_92 = {1{`RANDOM}};
  slowWakeupMatchVec_5_2 = _RAND_92[3:0];
  _RAND_93 = {1{`RANDOM}};
  slowWakeupMatchVec_6_0 = _RAND_93[3:0];
  _RAND_94 = {1{`RANDOM}};
  slowWakeupMatchVec_6_1 = _RAND_94[3:0];
  _RAND_95 = {1{`RANDOM}};
  slowWakeupMatchVec_6_2 = _RAND_95[3:0];
  _RAND_96 = {1{`RANDOM}};
  slowWakeupMatchVec_7_0 = _RAND_96[3:0];
  _RAND_97 = {1{`RANDOM}};
  slowWakeupMatchVec_7_1 = _RAND_97[3:0];
  _RAND_98 = {1{`RANDOM}};
  slowWakeupMatchVec_7_2 = _RAND_98[3:0];
  _RAND_99 = {1{`RANDOM}};
  dataArray_io_multiWrite_0_enable_REG = _RAND_99[0:0];
  _RAND_100 = {2{`RANDOM}};
  dataArray_io_multiWrite_0_data_r = _RAND_100[63:0];
  _RAND_101 = {1{`RANDOM}};
  dataArray_io_multiWrite_1_enable_REG = _RAND_101[0:0];
  _RAND_102 = {2{`RANDOM}};
  dataArray_io_multiWrite_1_data_r = _RAND_102[63:0];
  _RAND_103 = {1{`RANDOM}};
  dataArray_io_multiWrite_2_enable_REG = _RAND_103[0:0];
  _RAND_104 = {2{`RANDOM}};
  dataArray_io_multiWrite_2_data_r = _RAND_104[63:0];
  _RAND_105 = {1{`RANDOM}};
  dataArray_io_multiWrite_3_enable_REG = _RAND_105[0:0];
  _RAND_106 = {2{`RANDOM}};
  dataArray_io_multiWrite_3_data_r = _RAND_106[63:0];
  _RAND_107 = {1{`RANDOM}};
  fastWakeupMatch_0_0_0 = _RAND_107[0:0];
  _RAND_108 = {1{`RANDOM}};
  fastWakeupMatch_0_1_0 = _RAND_108[0:0];
  _RAND_109 = {1{`RANDOM}};
  fastWakeupMatch_0_2_0 = _RAND_109[0:0];
  _RAND_110 = {1{`RANDOM}};
  fastWakeupMatch_1_0_0 = _RAND_110[0:0];
  _RAND_111 = {1{`RANDOM}};
  fastWakeupMatch_1_1_0 = _RAND_111[0:0];
  _RAND_112 = {1{`RANDOM}};
  fastWakeupMatch_1_2_0 = _RAND_112[0:0];
  _RAND_113 = {1{`RANDOM}};
  fastWakeupMatch_2_0_0 = _RAND_113[0:0];
  _RAND_114 = {1{`RANDOM}};
  fastWakeupMatch_2_1_0 = _RAND_114[0:0];
  _RAND_115 = {1{`RANDOM}};
  fastWakeupMatch_2_2_0 = _RAND_115[0:0];
  _RAND_116 = {1{`RANDOM}};
  fastWakeupMatch_3_0_0 = _RAND_116[0:0];
  _RAND_117 = {1{`RANDOM}};
  fastWakeupMatch_3_1_0 = _RAND_117[0:0];
  _RAND_118 = {1{`RANDOM}};
  fastWakeupMatch_3_2_0 = _RAND_118[0:0];
  _RAND_119 = {1{`RANDOM}};
  fastWakeupMatch_4_0_0 = _RAND_119[0:0];
  _RAND_120 = {1{`RANDOM}};
  fastWakeupMatch_4_1_0 = _RAND_120[0:0];
  _RAND_121 = {1{`RANDOM}};
  fastWakeupMatch_4_2_0 = _RAND_121[0:0];
  _RAND_122 = {1{`RANDOM}};
  fastWakeupMatch_5_0_0 = _RAND_122[0:0];
  _RAND_123 = {1{`RANDOM}};
  fastWakeupMatch_5_1_0 = _RAND_123[0:0];
  _RAND_124 = {1{`RANDOM}};
  fastWakeupMatch_5_2_0 = _RAND_124[0:0];
  _RAND_125 = {1{`RANDOM}};
  fastWakeupMatch_6_0_0 = _RAND_125[0:0];
  _RAND_126 = {1{`RANDOM}};
  fastWakeupMatch_6_1_0 = _RAND_126[0:0];
  _RAND_127 = {1{`RANDOM}};
  fastWakeupMatch_6_2_0 = _RAND_127[0:0];
  _RAND_128 = {1{`RANDOM}};
  fastWakeupMatch_7_0_0 = _RAND_128[0:0];
  _RAND_129 = {1{`RANDOM}};
  fastWakeupMatch_7_1_0 = _RAND_129[0:0];
  _RAND_130 = {1{`RANDOM}};
  fastWakeupMatch_7_2_0 = _RAND_130[0:0];
  _RAND_131 = {1{`RANDOM}};
  s2_issuePtr_0 = _RAND_131[2:0];
  _RAND_132 = {1{`RANDOM}};
  s2_issuePtrOH_0 = _RAND_132[7:0];
  _RAND_133 = {1{`RANDOM}};
  s2_first_issue_0 = _RAND_133[0:0];
  _RAND_134 = {1{`RANDOM}};
  s2_all_src_ready_0 = _RAND_134[0:0];
  _RAND_135 = {1{`RANDOM}};
  data_uop_robIdx_flag = _RAND_135[0:0];
  _RAND_136 = {1{`RANDOM}};
  data_uop_robIdx_value = _RAND_136[4:0];
  _RAND_137 = {1{`RANDOM}};
  data_uop_ctrl_rfWen = _RAND_137[0:0];
  _RAND_138 = {1{`RANDOM}};
  data_uop_ctrl_fpWen = _RAND_138[0:0];
  _RAND_139 = {1{`RANDOM}};
  data_uop_ctrl_fpu_isAddSub = _RAND_139[0:0];
  _RAND_140 = {1{`RANDOM}};
  data_uop_ctrl_fpu_typeTagIn = _RAND_140[0:0];
  _RAND_141 = {1{`RANDOM}};
  data_uop_ctrl_fpu_fmaCmd = _RAND_141[1:0];
  _RAND_142 = {1{`RANDOM}};
  data_uop_ctrl_fpu_ren3 = _RAND_142[0:0];
  _RAND_143 = {1{`RANDOM}};
  data_uop_ctrl_fpu_rm = _RAND_143[2:0];
  _RAND_144 = {1{`RANDOM}};
  data_uop_pdest = _RAND_144[5:0];
  _RAND_145 = {1{`RANDOM}};
  midFinished2T2_0 = _RAND_145[0:0];
  _RAND_146 = {1{`RANDOM}};
  midIssuePtrT2_0 = _RAND_146[2:0];
  _RAND_147 = {1{`RANDOM}};
  midIssuePtrOHT1_0 = _RAND_147[7:0];
  _RAND_148 = {1{`RANDOM}};
  midIssuePtrT1_0 = _RAND_148[2:0];
  _RAND_149 = {1{`RANDOM}};
  midFinished2T1_0 = _RAND_149[0:0];
  _RAND_150 = {1{`RANDOM}};
  midIssuePtrOHT2_0 = _RAND_150[7:0];
  _RAND_151 = {1{`RANDOM}};
  hasCond12Reg = _RAND_151[0:0];
  _RAND_152 = {1{`RANDOM}};
  io_fmaMid_0_in_bits_rfp_prod_sign = _RAND_152[0:0];
  _RAND_153 = {1{`RANDOM}};
  io_fmaMid_0_in_bits_rfp_prod_exp = _RAND_153[10:0];
  _RAND_154 = {4{`RANDOM}};
  io_fmaMid_0_in_bits_rfp_prod_sig = _RAND_154[104:0];
  _RAND_155 = {1{`RANDOM}};
  io_fmaMid_0_in_bits_rinter_flags_isNaN = _RAND_155[0:0];
  _RAND_156 = {1{`RANDOM}};
  io_fmaMid_0_in_bits_rinter_flags_isInf = _RAND_156[0:0];
  _RAND_157 = {1{`RANDOM}};
  io_fmaMid_0_in_bits_rinter_flags_isInv = _RAND_157[0:0];
  _RAND_158 = {1{`RANDOM}};
  io_perf_0_value_REG = _RAND_158[0:0];
  _RAND_159 = {1{`RANDOM}};
  io_perf_0_value_REG_1 = _RAND_159[0:0];
`endif // RANDOMIZE_REG_INIT
  if (reset) begin
    validAfterAllocate = 8'h0;
  end
  if (reset) begin
    valid = 1'h0;
  end
  if (reset) begin
    hasCond12Reg = 1'h0;
  end
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule

