package xiangshan.backend.fu.wrapper

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import chisel3.util.experimental.decode.TruthTable
import utils.XSError
import xiangshan.backend.fu.FuConfig
import xiangshan.backend.fu.vector.Bundles.{VLmul, VSew, ma}
import xiangshan.backend.fu.vector.utils.VecDataSplitModule
import xiangshan.backend.fu.vector.{Mgu, VecInfo, VecPipedFuncUnit}
import yunsuan.VfpuType
import yunsuan.VfcvtType
import yunsuan.vector.VectorConvert.VectorCvt


class VCVT(cfg: FuConfig)(implicit p: Parameters) extends VecPipedFuncUnit(cfg) {
  XSError(io.in.valid && io.in.bits.ctrl.fuOpType === VfpuType.dummy, "Vfcvt OpType not supported")

  // params alias
  private val dataWidth = cfg.dataBits
  private val dataWidthOfDataModule = 64
  private val numVecModule = dataWidth / dataWidthOfDataModule

  // io alias
  private val opcode = fuOpType(7,0)
  private val sew = vsew
  private val rtz_rm = opcode(2) & opcode(1)
  private val rod_rm = opcode(2) & !opcode(1) & opcode(0)
  private val rm = Mux1H(
    Seq(!rtz_rm & !rod_rm,
      rtz_rm,
      rod_rm),
    Seq(frm,
      1.U,
      6.U)
  )
  private val lmul = vlmul

  val widen = opcode(4,3) // 0->single 1->widen 2->narrow
  val isSingleCvt = !widen(1) & !widen(0)
  val isWidenCvt  = !widen(1) & widen(0)
  val isNarrowCvt =  widen(1) & !widen(0)


  val output1H = Wire(UInt(4.W))
  output1H := chisel3.util.experimental.decode.decoder(
    widen ## sew,
    TruthTable(
      Seq(
        BitPat("b00_01") -> BitPat("b0010"), // 16
        BitPat("b00_10") -> BitPat("b0100"), // 32
        BitPat("b00_11") -> BitPat("b1000"), // 64

        BitPat("b01_00") -> BitPat("b0010"), // 16
        BitPat("b01_01") -> BitPat("b0100"), // 32
        BitPat("b01_10") -> BitPat("b1000"), // 64

        BitPat("b10_00") -> BitPat("b0001"), // 8
        BitPat("b10_01") -> BitPat("b0010"), // 16
        BitPat("b10_10") -> BitPat("b0100"), // 32
      ),
      BitPat("b0000") // f8, don't exist
    )
  )
  dontTouch(output1H)
  val outputWidth1H = output1H
  val outEew = Mux1H(output1H, Seq(0,1,2,3).map(i => i.U))

  // modules
  private val vfcvt = Module(new VectorCvtTop(dataWidth, dataWidthOfDataModule))
  private val mgu = Module(new Mgu(dataWidth))

  /**
   * [[vfcvt]]'s in connection
   */
  vfcvt.io.uopIdx := vuopIdx(0)
  vfcvt.io.src := vs2
  vfcvt.io.opType := opcode
  vfcvt.io.sew := sew
  vfcvt.io.rm := rm
  vfcvt.io.outputWidth1H := outputWidth1H
  vfcvt.io.isWiden := isWidenCvt
  vfcvt.io.isNarrow := isNarrowCvt
  val vfcvtResult = Wire(UInt(dataWidth.W))
  vfcvtResult := vfcvt.io.result.asTypeOf(vfcvtResult)
  val vfcvtFflags = Wire(UInt((numVecModule*20).W))
  vfcvtFflags := vfcvt.io.fflags.asTypeOf(vfcvtFflags)

  val eNum1H = chisel3.util.experimental.decode.decoder(sew ## (isWidenCvt || isNarrowCvt),
    TruthTable(
      Seq(                // 8, 4, 2, 1
        //BitPat("b000") -> BitPat("b1000"), // 8
        BitPat("b001") -> BitPat("b1000"), // 8
        BitPat("b010") -> BitPat("b1000"), // 8
        BitPat("b011") -> BitPat("b0100"), // 4
        BitPat("b100") -> BitPat("b0100"), // 4
        BitPat("b101") -> BitPat("b0010"), // 2
        BitPat("b110") -> BitPat("b0010"), // 2
        //BitPat("b111") -> BitPat("b0010"), // 2
      ),
      BitPat.N(4)
    )
  )
  val eNumMax1H = Mux(lmul.head(1).asBool, eNum1H >> ((~lmul.tail(1)).asUInt + 1.U), eNum1H.tail(1) << lmul).asUInt(6, 0)
  val eNumMax = Mux1H(eNumMax1H, Seq(1,2,4,8,16,32,64).map(i => i.U))
  val eNumEffect = Mux(vl > eNumMax, eNumMax, vl)

  val fflagsAll = Wire(Vec(4 * numVecModule, UInt(5.W)))
  fflagsAll := vfcvtFflags.asTypeOf(fflagsAll)
  // mask, vl和lmul => 某个输入的向量元素是否有效
  val mask = Mux1H(eNum1H, Seq(1,2,4,8).map(num => (srcMask >> vuopIdx * num.U)(num-1, 0)))
  val fflagsEn = Wire(Vec(4 * numVecModule, Bool()))
  fflagsEn := mask.asBools.zipWithIndex.map{
    case(mask, i) => mask & (eNumEffect > Mux1H(eNum1H, Seq(1,2,4,8).map(num => vuopIdx * num.U + i.U)))
  }
  val fflagsEnCycle2 = RegNext(RegNext(fflagsEn))
  val fflags = fflagsEnCycle2.zip(fflagsAll).map{
    case(en, fflag) => Mux(en, fflag, 0.U(5.W))
  }.reduce(_ | _)
  io.out.bits.res.fflags.get := fflags

  /**
   * [[mgu]]'s in connection
   */

  private val needNoMask = outVecCtrl.fpu.isFpToVecInst
  val maskToMgu = Mux(needNoMask, allMaskTrue, outSrcMask)

  val resultDataUInt = Wire(UInt(dataWidth.W))
  resultDataUInt := vfcvtResult
  mgu.io.in.vd := resultDataUInt
  mgu.io.in.oldVd := outOldVd
  mgu.io.in.mask := maskToMgu
  mgu.io.in.info.ta := outVecCtrl.vta
  mgu.io.in.info.ma := outVecCtrl.vma
  mgu.io.in.info.vl := Mux(outVecCtrl.fpu.isFpToVecInst, 1.U, outVl)
  mgu.io.in.info.vlmul := outVecCtrl.vlmul
  mgu.io.in.info.valid := io.out.valid
  mgu.io.in.info.vstart := Mux(outVecCtrl.fpu.isFpToVecInst, 0.U, outVecCtrl.vstart)
  mgu.io.in.info.eew := outEew
  mgu.io.in.info.vsew := outVecCtrl.vsew
  mgu.io.in.info.vdIdx := outVecCtrl.vuopIdx
  mgu.io.in.info.narrow := isNarrowCvt
  mgu.io.in.info.dstMask := outVecCtrl.isDstMask

  io.out.bits.res.data := mgu.io.out.vd

}

// uopindex, 1: high64, 0: low64
class VectorCvtTop(vlen :Int, xlen :Int) extends Module {
  val io = IO(new Bundle() {
    val uopIdx = Input(UInt(1.W))
    val src = Input(UInt(vlen.W))
    val opType = Input(UInt(8.W))
    val sew = Input(UInt(2.W))
    val rm = Input(UInt(3.W))
    val outputWidth1H = Input(UInt(4.W))
    val isWiden = Input(Bool())
    val isNarrow = Input(Bool())

    val result = Output(UInt(vlen.W))
    val fflags = Output(UInt(40.W))
  })

  val in0 = Mux(io.isWiden,
    Mux(io.uopIdx(0), io.src.tail(32).head(32), io.src.tail(96)),
    io.src.tail(64)
  )
  val in1 = Mux(io.isWiden,
    Mux(io.uopIdx(0), io.src.head(32), io.src.tail(64).head(32)),
    io.src.head(64)
  )

  val vectorCvt0 = Module(new VectorCvt(xlen))
  vectorCvt0.io.src := in0
  vectorCvt0.io.opType := io.opType
  vectorCvt0.io.sew := io.sew
  vectorCvt0.io.rm := io.rm

  val vectorCvt1 = Module(new VectorCvt(xlen))
  vectorCvt1.io.src := in1
  vectorCvt1.io.opType := io.opType
  vectorCvt1.io.sew := io.sew
  vectorCvt1.io.rm := io.rm

  val isNarrowCycle2 = RegNext(RegNext(io.isNarrow))
  val outputWidth1HCycle2 = RegNext(RegNext(io.outputWidth1H))

  //cycle2
  io.result := Mux(isNarrowCycle2,
    vectorCvt1.io.result.tail(32) ## vectorCvt0.io.result.tail(32),
    vectorCvt1.io.result ## vectorCvt0.io.result
  )
  io.fflags := Mux1H(outputWidth1HCycle2, Seq(
    vectorCvt1.io.fflags ## vectorCvt0.io.fflags,
    Mux(isNarrowCycle2, vectorCvt1.io.fflags.tail(10) ## vectorCvt0.io.fflags.tail(10),
      vectorCvt1.io.fflags ## vectorCvt0.io.fflags),
    Mux(isNarrowCycle2, vectorCvt1.io.fflags(4,0) ## vectorCvt0.io.fflags(4,0),
      vectorCvt1.io.fflags.tail(10) ## vectorCvt0.io.fflags.tail(10)),
    vectorCvt1.io.fflags(4,0) ## vectorCvt0.io.fflags(4,0)
  ))


}
