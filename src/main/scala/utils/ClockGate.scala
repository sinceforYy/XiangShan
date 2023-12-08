package utils

import chisel3._
import chisel3.util.{HasBlackBoxPath, HasBlackBoxResource}
import org.chipsalliance.cde.config.{Field, Parameters}

import java.nio.file.{Files, Paths}

case object ClockGateImpl extends Field[() => ClockGate](() => new EICG_wrapper)
case object ClockGateModelFile extends Field[Option[String]](None)

abstract class ClockGate extends BlackBox
  with HasBlackBoxResource with HasBlackBoxPath {
  val io = IO(new Bundle {
    val TE = Input(Bool())
    val E = Input(Bool())
    val CK = Input(Clock())
    val Q = Output(Clock())
  })

  def addVerilogResource(vsrc: String): Unit = {
    if (Files.exists(Paths.get(vsrc)))
      addPath(vsrc)
    else
      addResource(vsrc)
  }
}

object ClockGate {
  def apply[T <: ClockGate](
      CK: Clock,
      E: Bool)(implicit p: Parameters): Clock = {
    val cg = Module(p(ClockGateImpl)())
    p(ClockGateModelFile).map(cg.addVerilogResource(_))

    cg.io.CK := CK
    cg.io.TE := false.B
    cg.io.E := E
    cg.io.Q
  }
}

// behavioral model of Integrated Clock Gating cell
class EICG_wrapper extends ClockGate