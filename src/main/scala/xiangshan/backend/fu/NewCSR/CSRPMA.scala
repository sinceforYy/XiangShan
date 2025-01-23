package xiangshan.backend.fu.NewCSR

import chisel3._
import chisel3.util._
import freechips.rocketchip.rocket.CSRs
import freechips.rocketchip.tile.XLen
import org.chipsalliance.cde.config.Parameters
import xiangshan.backend.fu.NewCSR.CSRDefines.{CSRRWField => RW, CSRWARLField => WARL}
import xiangshan.backend.fu.NewCSR.CSRFunc._
import xiangshan.backend.fu.PMAConfigEntry
import xiangshan.{HasPMParameters, PMParameKey}
import CSRConfig._
import system.SoCParamsKey

import scala.collection.immutable.SeqMap

trait CSRPMA { self: NewCSR =>
  val pmacfg: Seq[CSRModule[_]] = Range(0, p(PMParameKey).NumPMA/8+1, 2).map(num =>
    Module(new CSRModule(s"Pmacfg$num") with HasPMACfgRSink {
      // read condition
      regOut := cfgRData(64*(num/2+1)-1, 64*num/2)
    })
      .setAddr(CSRs.pmacfg0 + num)
  )

  // every pmacfg has 8 cfgs
  val pmacfgs: Seq[CSRModule[_]] = Range(0, p(PMParameKey).NumPMA).map(num =>
    Module(new CSRModule(s"Pma$num"+"cfg", new PMACfgInitBundle(num)))
  )

  // dirty code
  val pmaaddr0: CSRModule[_] = Module(new CSRModule(s"Pmaaddr0", new PMAAddr0InitBundle))
    .setAddr(CSRs.pmpaddr0)
  val pmaaddr1: CSRModule[_] = Module(new CSRModule(s"Pmaaddr1", new PMAAddr1InitBundle))
    .setAddr(CSRs.pmpaddr1)
  val pmaaddr2: CSRModule[_] = Module(new CSRModule(s"Pmaaddr2", new PMAAddr2InitBundle))
    .setAddr(CSRs.pmpaddr2)
  val pmaaddr3: CSRModule[_] = Module(new CSRModule(s"Pmaaddr3", new PMAAddr3InitBundle))
    .setAddr(CSRs.pmpaddr3)
  val pmaaddr4: CSRModule[_] = Module(new CSRModule(s"Pmaaddr4", new PMAAddr4InitBundle))
    .setAddr(CSRs.pmpaddr4)
  val pmaaddr5: CSRModule[_] = Module(new CSRModule(s"Pmaaddr5", new PMAAddr5InitBundle))
    .setAddr(CSRs.pmpaddr5)
  val pmaaddr6: CSRModule[_] = Module(new CSRModule(s"Pmaaddr6", new PMAAddr6InitBundle))
    .setAddr(CSRs.pmpaddr6)
  val pmaaddr7: CSRModule[_] = Module(new CSRModule(s"Pmaaddr7", new PMAAddr7InitBundle))
    .setAddr(CSRs.pmpaddr7)
  val pmaaddr8: CSRModule[_] = Module(new CSRModule(s"Pmaaddr8", new PMAAddr8InitBundle))
    .setAddr(CSRs.pmpaddr8)
  val pmaaddr9: CSRModule[_] = Module(new CSRModule(s"Pmaaddr9", new PMAAddr9InitBundle))
    .setAddr(CSRs.pmpaddr9)
  val pmaaddr10: CSRModule[_] = Module(new CSRModule(s"Pmaaddr10", new PMAAddr10InitBundle))
    .setAddr(CSRs.pmpaddr10)
  val pmaaddr11: CSRModule[_] = Module(new CSRModule(s"Pmaaddr11", new PMAAddr11InitBundle))
    .setAddr(CSRs.pmpaddr11)
  val pmaaddr12: CSRModule[_] = Module(new CSRModule(s"Pmaaddr12", new PMAAddr12InitBundle))
    .setAddr(CSRs.pmpaddr12)
  val pmaaddr13: CSRModule[_] = Module(new CSRModule(s"Pmaaddr13", new PMAAddr13InitBundle))
    .setAddr(CSRs.pmpaddr13)
  val pmaaddr14: CSRModule[_] = Module(new CSRModule(s"Pmaaddr14", new PMAAddr14InitBundle))
    .setAddr(CSRs.pmpaddr14)
  val pmaaddr15: CSRModule[_] = Module(new CSRModule(s"Pmaaddr15", new PMAAddr15InitBundle))
    .setAddr(CSRs.pmpaddr15)


  val pmaaddr: Seq[CSRModule[_]] = Seq(
    pmaaddr0, pmaaddr1, pmaaddr2, pmaaddr3, pmaaddr4, pmaaddr5, pmaaddr6, pmaaddr7,
    pmaaddr8, pmaaddr9, pmaaddr10, pmaaddr11, pmaaddr12, pmaaddr13, pmaaddr14, pmaaddr15
  )

//  val pmaaddr: Seq[CSRModule[_]] = Range(0, p(PMParameKey).NumPMA).map(num =>
//    Module(new CSRModule(s"Pmaaddr$num", new PMAAddrInitBundle(num)) with HasPMAAddrSink {
//      // read condition
//      rdata := addrRData(num)
//    })
//      .setAddr(CSRs.pmaaddr0 + num)
//  )

  val pmaCSRMods: Seq[CSRModule[_]] = pmacfg ++ pmaaddr

  val pmaCSRMap: SeqMap[Int, (CSRAddrWriteBundle[_], UInt)] = SeqMap.from(
    pmaCSRMods.map(csr => csr.addr -> (csr.w -> csr.rdata)).iterator
  )

  val pmaCSROutMap: SeqMap[Int, UInt] = SeqMap.from(
    pmpCSRMods.map(csr => csr.addr -> csr.regOut.asInstanceOf[CSRBundle].asUInt).iterator
  )

  private val pmaCfgRead = Cat(pmacfgs.map(_.rdata(7, 0)).reverse)

  pmaCSRMods.foreach { mod =>
    mod match {
      case m: HasPMACfgRSink =>
        m.cfgRData := pmaCfgRead
      case _ =>
    }
  }
}

class PMACfgInitBundle(num: Int)(implicit val p: Parameters) extends PMACfgBundle with PMAInit {
  override val R      = WARL(0, wNoFilter).withReset(pmaInit(num).r.B)
  override val W      = WARL(1, wNoFilter).withReset(pmaInit(num).w.B)
  override val X      = WARL(2, wNoFilter).withReset(pmaInit(num).x.B)
  override val A      = PMPCfgAField(4, 3, wNoFilter).withReset(if (pmaInit(num).a > 0) pmaInit(num).a.U else 0.U)
  override val ATOMIC = WARL(5, wNoFilter).withReset(pmaInit(num).atomic.B)
  override val C      = WARL(6, wNoFilter).withReset(pmaInit(num).c.B)
  override val L      = PMPCfgLField(7, wNoFilter).withReset(pmaInit(num).l.B)
}

class PMACfgBundle extends PMPCfgBundle {
  override val ATOMIC = WARL(5, wNoFilter).withReset(false.B)
  override val C      = WARL(6, wNoFilter).withReset(false.B)
}

class PMAAddr0InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
//  println(s"pmaInit: $pmaInit")
//  require(pmaInit.nonEmpty, "PmaInit cannot be empty")
//  val addrInit = pmaInit.head
//  println(s"PMAAddr0Init, ${addrInit}")
//  val addr = WireInit(0.U((PMPAddrWidth-PMPOffBits).W))
//  addr := genAddr(addrInit)
//  println(s"addr, ${addr}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0, pmaInitAddr.head).withReset(pmaInitAddr.head)
}

class PMAAddr1InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(1).head
  println(s"PMAAddr1Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr2InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(2).head
  println(s"PMAAddr2Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr3InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(3).head
  println(s"PMAAddr3Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr4InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(4).head
  println(s"PMAAddr4Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr5InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(5).head
  println(s"PMAAddr5Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0, pmaInitAddr.drop(5).head).withReset(pmaInitAddr.drop(5).head)
}

class PMAAddr6InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(6).head
  println(s"PMAAddr6Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr7InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(7).head
  println(s"PMAAddr7Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr8InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(8).head
  println(s"PMAAddr8Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr9InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(9).head
  println(s"PMAAddr9Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr10InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(10).head
  println(s"PMAAddr10Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr11InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(11).head
  println(s"PMAAddr11Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr12InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(12).head
  println(s"PMAAddr12Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr13InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(13).head
  println(s"PMAAddr13Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr14InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(14).head
  println(s"PMAAddr14Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

class PMAAddr15InitBundle(implicit val p: Parameters) extends PMPAddrBundle with PMAInit with PMAReadWrite {
  val addrInit = pmaInit.drop(15).head
  println(s"PMAAddr15Init, ${addrInit}")
  override val ADDRESS = RW(PMPAddrWidth-PMPOffBits-1, 0).withReset(genAddr(addrInit).U)
}

trait HasPMACfgRSink { self: CSRModule[_] =>
  val cfgRData = IO(Input(UInt((p(PMParameKey).NumPMA/8 * p(XLen)).W)))
}

trait HasPMAAddrSink { self: CSRModule[_] =>
  val addrRData = IO(Input(Vec(p(PMParameKey).NumPMA, UInt(64.W))))
}

trait PMAInit extends HasPMParameters with PMAReadWrite {
  def pmaInit: Seq[PMAConfigEntry] = (PMAConfigs ++ Seq.fill(NumPMA-PMAConfigs.length)(PMAConfigEntry(0))).reverse
  def pmaInitAddr: Seq[UInt] = pmaInit.map(genAddr(_).U)

}
