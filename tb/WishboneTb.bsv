package WishboneTb;

import Vector::*;
import StmtFSM::*;
import ConfigReg::*;
import RegIf::*;
import Wishbone::*;

// 零等待假外设：0x04 读写（按选通合并）· 0x08 读回写过几次 · 0x0C 一律回错
module mkWbPeriph(RegIf#(8, 32));
  Reg#(Bit#(32)) v  <- mkReg(0);
  Reg#(Bit#(16)) wc <- mkReg(0);

  method ActionValue#(RegRsp#(32)) access(RegReq#(8, 32) r);
    Bool     er = r.addr == 8'h0C;
    Bit#(32) rd = r.addr == 8'h08 ? zeroExtend(wc) : v;
    if (r.write && !er) begin
      wc <= wc + 1;
      if (r.addr == 8'h04) v <= applyStrb(v, r.wdata, r.wstrb);
    end
    return RegRsp { rdata: rd, err: er };
  endmethod
endmodule

// 会停顿的同一个外设，压 n 拍才答。端口 0 归规则，端口 1 归方法
module mkWbSlow#(Integer n)(RegTarget#(8, 32));
  Reg#(Bit#(8))        cnt[2]  <- mkCReg(2, 0);
  Reg#(Bool)           busy[2] <- mkCReg(2, False);
  Reg#(Bool)           ansV[2] <- mkCReg(2, False);
  Reg#(RegReq#(8, 32)) q[2]    <- mkCReg(2, unpack(0));
  Reg#(RegRsp#(32))    ans[2]  <- mkCReg(2, unpack(0));
  Reg#(Bit#(32))       v       <- mkReg(0);
  Reg#(Bit#(16))       wc      <- mkReg(0);

  rule tick;
    if (busy[0] && cnt[0] == 0) begin
      let r = q[0];
      Bool er = r.addr == 8'h0C;
      busy[0] <= False;
      ansV[0] <= True;
      ans[0]  <= RegRsp { rdata: r.addr == 8'h08 ? zeroExtend(wc) : v, err: er };
      if (r.write && !er) begin
        wc <= wc + 1;
        if (r.addr == 8'h04) v <= applyStrb(v, r.wdata, r.wstrb);
      end
    end else begin
      if (busy[0]) cnt[0] <= cnt[0] - 1;
      ansV[0] <= False;
    end
  endrule

  method Action req(Bool valid, RegReq#(8, 32) r);
    if (valid && !busy[1] && !ansV[1]) begin
      busy[1] <= True;
      cnt[1]  <= fromInteger(n);
      q[1]    <= r;
    end
  endmethod
  method Bool ready = !busy[1] && !ansV[1];
  method Bool rspValid = ansV[1];
  method RegRsp#(32) rsp = ans[1];
endmodule

// 一个流水线模式的主机：STALL 为低就把下一笔摆上去，不等 ACK（3.1.3.2）。从设备输出都出自寄存器，
// 读 STALL/ACK 与驱动引脚写在同一条规则里。主机寄存器端口 0 归这条规则，端口 1 归命令序列。
(* synthesize *)
module mkWishboneTb(Empty);
  RegIf#(8, 32)       dev  <- mkWbPeriph;
  RegTarget#(8, 32)   sdev <- mkWbSlow(2);
  WbSlavePins#(8, 32) sa   <- mkWbBind(dev);
  WbSlavePins#(8, 32) sb   <- mkWbBindT(sdev);

  Reg#(Bool)      onB      <- mkReg(False);
  Reg#(Bool)      cyc[2]   <- mkCReg(2, False);
  Reg#(Bool)      stb[2]   <- mkCReg(2, False);
  Reg#(UInt#(8))  begun[2] <- mkCReg(2, 0);
  Reg#(UInt#(8))  done[2]  <- mkCReg(2, 0);
  Reg#(Bool)      bad[2]   <- mkCReg(2, False);
  Reg#(Bool)      we       <- mkReg(False);
  Reg#(Bit#(8))   adr      <- mkReg(0);
  Reg#(Bit#(32))  dat      <- mkReg(0);
  Reg#(Bit#(4))   sel      <- mkReg(0);
  // 主机规则写、命令序列读：用 ConfigReg，两边才排得出先后（否则 G0010，命令序列饿死）
  Vector#(8, Reg#(Bit#(32))) got <- replicateM(mkConfigReg(0));
  Vector#(8, Reg#(Bool))     gotErr <- replicateM(mkConfigReg(False));
  Reg#(UInt#(16)) cycles   <- mkReg(0);
  Reg#(Bool)      saidOrphan <- mkReg(False);
  Reg#(Bool)      saidBoth   <- mkReg(False);

  rule master;
    Bool     st = onB ? sb.stall : sa.stall;
    Bool     ak = onB ? sb.ack   : sa.ack;
    Bool     er = onB ? sb.err   : sa.err;
    Bit#(32) rd = onB ? sb.rdata : sa.rdata;
    sa.req(cyc[0] && !onB, stb[0] && !onB, we, adr, dat, sel);
    sb.req(cyc[0] && onB,  stb[0] && onB,  we, adr, dat, sel);

    if (cyc[0] && stb[0] && !st) begin
      stb[0] <= False;
      begun[0] <= begun[0] + 1;
    end
    Bool fail = False;
    if (ak || er) begin
      if (done[0] < 8) begin
        got[done[0]] <= rd;
        gotErr[done[0]] <= er;
      end
      done[0] <= done[0] + 1;
      if (done[0] >= begun[0] && !saidOrphan) begin
        $display("FAIL ACK or ERR came with no transfer outstanding");
        saidOrphan <= True; fail = True;
      end
    end
    if (ak && er && !saidBoth) begin
      $display("FAIL ACK and ERR were high in the same cycle");
      saidBoth <= True; fail = True;
    end
    if (fail) bad[0] <= True;
  endrule

  function Stmt put(Bool w, Bit#(8) a, Bit#(32) d, Bit#(4) s) = seq
    action we <= w; adr <= a; dat <= d; sel <= s; stb[1] <= True; endaction
    await(!stb[1]);
  endseq;

  function Action fresh = action
    cyc[1] <= True; begun[1] <= 0; done[1] <= 0;
  endaction;

  function Stmt answers(UInt#(8) n, String what) = seq
    action cycles <= 0; endaction
    while (done[1] < n && cycles < 200) action cycles <= cycles + 1; endaction
    action
      if (done[1] != n) begin
        $display("FAIL %s: %0d of %0d transfers were answered", what, done[1], n);
        bad[1] <= True;
      end
      cyc[1] <= False;
    endaction
  endseq;

  function Stmt want(Integer i, Bit#(32) v, Bool e, String what) = seq
    action
      if (got[i] != v || gotErr[i] != e) begin
        $display("FAIL %s: DAT_O %08h ERR %0d, want %08h and %0d", what, got[i], gotErr[i], v, e);
        bad[1] <= True;
      end
    endaction
  endseq;

  Stmt suite = seq
    // 四笔背靠背摆上去，下一笔不等上一笔的 ACK
    fresh;
    put(True,  8'h04, 32'h11111111, 4'hF);
    put(True,  8'h04, 32'h22222222, 4'hF);
    put(False, 8'h04, 0, 4'hF);
    put(False, 8'h08, 0, 4'hF);
    answers(4, "four pipelined transfers");
    want(2, 32'h22222222, False, "a pipelined read after two pipelined writes");
    want(3, 2, False, "the write count after two writes");

    fresh;
    put(True,  8'h0C, 32'hFFFFFFFF, 4'hF);
    put(False, 8'h08, 0, 4'hF);
    answers(2, "a write to the address that errors and a read");
    want(0, 0, True, "a write to the address that always errors");
    action
      if (got[1] != 2) begin $display("FAIL the errored write counted as a write: count %0d", got[1]); bad[1] <= True; end
    endaction

    fresh;
    put(True,  8'h04, 32'hAABBCCDD, 4'b0101);
    put(False, 8'h04, 0, 4'hF);
    answers(2, "a half-strobed write and a read");
    want(1, 32'h22BB22DD, False, "the read after a write strobing bytes 0 and 2");

    // CYC 为低时 STB 不算数
    action cyc[1] <= False; we <= True; adr <= 8'h04; dat <= 32'hDEADBEEF; sel <= 4'hF; stb[1] <= True; endaction
    delay(4);
    action stb[1] <= False; endaction
    fresh;
    put(False, 8'h04, 0, 4'hF);
    put(False, 8'h08, 0, 4'hF);
    answers(2, "reads after STB was raised without CYC");
    want(0, 32'h22BB22DD, False, "the register after STB without CYC");
    want(1, 3, False, "the write count after STB without CYC");
  endseq;

  Stmt test = seq
    suite;
    action onB <= True; endaction
    suite;
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);
  Reg#(UInt#(16)) cyc16 <- mkReg(0);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule count;
    cyc16 <= cyc16 + 1;
    if (cyc16 > 5000) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    if (bad[1]) $display("FAILED");
    else $display("PASS wishbone: pipelined transfers start only with CYC, STB and STALL low, each gets one ACK "
                  + "or ERR in order, strobes merge, and a stalling target holds the next transfer with STALL");
    $finish(bad[1] ? 1 : 0);
  endrule
endmodule

endpackage
