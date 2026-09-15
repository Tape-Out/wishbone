package Wishbone;

import RegIf::*;

// Wishbone B4（OpenCores 2010）的流水线模式：CYC、STB 为高且 STALL 为低的那一拍，一笔传输就开始了
// （RULE 3.57、3.58）；主机随时收 ACK（RULE 3.59）；流水线模式下 ACK、ERR 不看同拍的 STB（OBSERVATION 3.10）；
// 读数据跟着 ACK 有效（3.1.3.2、RULE 3.65）。答复一律打一拍再出去。

interface WbSlavePins#(numeric type aw, numeric type dw);
  (* always_ready, always_enabled, prefix = "" *)
  method Action req((* port = "cyc_i" *) Bool               cyc,
                    (* port = "stb_i" *) Bool               stb,
                    (* port = "we_i" *)  Bool               we,
                    (* port = "adr_i" *) Bit#(aw)           adr,
                    (* port = "dat_i" *) Bit#(dw)           dat,
                    (* port = "sel_i" *) Bit#(TDiv#(dw, 8)) sel);
  (* always_ready, result = "ack_o" *)   method Bool     ack;
  (* always_ready, result = "err_o" *)   method Bool     err;
  (* always_ready, result = "stall_o" *) method Bool     stall;
  (* always_ready, result = "dat_o" *)   method Bit#(dw) rdata;
endinterface

// 零等待目标：每拍都收得下，STALL 恒低；一笔在开始那一拍就访问目标，下一拍出 ACK 或 ERR
module mkWbBind#(RegIf#(aw, dw) rf)(WbSlavePins#(aw, dw));
  Reg#(Bool)     ackR <- mkReg(False);
  Reg#(Bool)     errR <- mkReg(False);
  Reg#(Bit#(dw)) datR <- mkReg(0);

  method Action req(Bool cyc, Bool stb, Bool we, Bit#(aw) adr, Bit#(dw) dat, Bit#(TDiv#(dw, 8)) sel);
    // CYC 为低时主机的其余信号都无效（3.1.2）
    if (cyc && stb) begin
      let x <- rf.access(RegReq { addr: adr, write: we, wdata: dat, wstrb: sel });
      ackR <= !x.err;
      errR <= x.err;
      // 出 ERR 那一拍 DAT_O 给 0：规范只认 ACK 那一拍的数据，出错时不把目标里的值带出去
      datR <= x.err ? 0 : x.rdata;
    end else begin
      ackR <= False;
      errR <= False;
    end
  endmethod

  method Bool     ack   = ackR;
  method Bool     err   = errR;
  method Bool     stall = False;
  method Bit#(dw) rdata = datR;
endmodule

// 会停顿的目标：手上有一笔没答完就把 STALL 拉高，主机顶着下一笔等（3.1.3.2）
module mkWbBindT#(RegTarget#(aw, dw) t)(WbSlavePins#(aw, dw));
  Wire#(Maybe#(RegReq#(aw, dw))) inW  <- mkBypassWire;
  Reg#(Maybe#(RegReq#(aw, dw)))  pend <- mkReg(tagged Invalid);
  Reg#(Bool)                     ackR <- mkReg(False);
  Reg#(Bool)                     errR <- mkReg(False);
  Reg#(Bit#(dw))                 datR <- mkReg(0);

  // 发与收分两条规则，道理同 hwcore 的 mkPipe
  rule send;
    t.req(isValid(pend), fromMaybe(unpack(0), pend));
  endrule

  rule step;
    Maybe#(RegReq#(aw, dw)) np = pend;
    Bool     na = False;
    Bool     ne = False;
    Bit#(dw) nd = datR;
    if (isValid(pend)) begin
      if (t.rspValid) begin
        let x = t.rsp;
        na = !x.err; ne = x.err; nd = x.err ? 0 : x.rdata;
        np = tagged Invalid;
      end
    end else if (inW matches tagged Valid .r)
      np = tagged Valid r;   // 这一拍 STALL 是低的，按 RULE 3.57、3.58 这一笔已经开始
    pend <= np;
    ackR <= na;
    errR <= ne;
    datR <= nd;
  endrule

  method Action req(Bool cyc, Bool stb, Bool we, Bit#(aw) adr, Bit#(dw) dat, Bit#(TDiv#(dw, 8)) sel);
    inW <= (cyc && stb) ? tagged Valid RegReq { addr: adr, write: we, wdata: dat, wstrb: sel }
                        : tagged Invalid;
  endmethod

  method Bool     ack   = ackR;
  method Bool     err   = errR;
  method Bool     stall = isValid(pend);
  method Bit#(dw) rdata = datR;
endmodule

endpackage
