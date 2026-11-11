#!/usr/bin/env python3
"""gen_board_roms.py - the AC701 matrix-engine-only board run's memory files, from the vector files (rows 5, 8, 16, 24).

The board shell (rtl/board) plays the host and the memory side for one GEMM from on-chip ROMs, exactly as
tb_me_top_s1 does in simulation: the debug-register sequence (CLR, WIN, GEMM, DUMP), the window stream, the
weight stream and the dump, checked on chip against the reference model's flag-free drain.

Eight cases, picked by sel[2:0] (DIP switches 0-2): sel[2] the layer (0: l0_q, 1: l29_q), sel[1:0] the rows
(0: 5, 1: 8, 2: 16, 3: 24 = two chats).

Usage:
  python golden/gen_board_roms.py --vec <dir holding r5, r8, r16, r24> --out <dir>
Files (in --out), all $readmemh text:
  bd_wt.mem      weight ROM, 512 bits a line: each layer's stream once (it is the same at every row count)
  bd_wt_b<b>.mem  the same in banks of 1,024 lines (b = 0, 1, ..., no leading zero) (bd_rom's BANK = 1024: Vivado's 1K x 36 BRAM mapping)
  bd_win.mem     window ROM, 512 bits a line: each case's loader stream (the constants beat, then the rows)
  bd_ref.mem     reference ROM, 512 bits a line: each case's expected output beats, in dump order (only the
                 beats holding the GEMM's outputs: 32 INT16 outputs a beat, channel 32b + j in bits 16j +: 16)
  bd_script.mem  script ROM, 64 bits a line: each case's register sequence (op 63:62, address 43:32, data 31:0;
                 op 0 write, 1 GO and wait, 2 read into capture slot data[1:0], 3 end)
  bd_case.mem    case table, 32 bits a line, 32 lines a case (field order: CASE_FIELDS)
  bd_cases.txt   the cases in words, with their sizes (for the log and the paper)
"""

import argparse
import os

ROWS = (5, 8, 16, 24)
LAYERS = ("l0_q", "l29_q")
ROW_BEATS = 96                      # dump beats per staging row: 1,536 words / 16
WIN_DDR, DUMP_DDR = 0x1000_0000, 0x2000_0000
A_CMD_LO, A_CMD_HI, A_GO = 0x100, 0x104, 0x108
A_STEP, A_LOGIT_LO, A_LOGIT_HI, A_CLASS = 0x110, 0x114, 0x118, 0x11C
A_WIN, A_WIN_ADDR, A_DUMP, A_DUMP_BEATS, A_DUMP_ADDR = 0x120, 0x124, 0x128, 0x12C, 0x130
A_SAT, A_CYCLES, A_BEATS = 0x134, 0x138, 0x13C
BUF_STG16, BUF_WIN = 0x2, 0x4
OP_WR, OP_GO, OP_RD, OP_END = 0, 1, 2, 3
CASE_FIELDS = ("script_base", "win_base", "win_beats", "wt_base", "wt_beats", "run_n", "run_k", "layer", "tag",
               "ref_base", "ref_beats", "dump_beats", "rel_lo", "rel_n", "exp_sat", "exp_arr_beats")
CASE_LINES = 32


def read_mem(path, bits):
    with open(path) as f:
        return [int(l, 16) for l in (x.strip() for x in f) if l and not l.startswith("//")]


def hexw(v, bits):
    return "%0*x" % (bits // 4, v & ((1 << bits) - 1))


def cmd_fields(c):
    """me_cmd_pkg::cmd_t, most significant field first."""
    return dict(op=(c >> 58) & 0x3F, flags=(c >> 48) & 0x3FF, abuf=(c >> 44) & 0xF, dbuf=(c >> 40) & 0xF,
                tag=(c >> 35) & 0x1F, iclass=(c >> 31) & 0xF, oclass=(c >> 27) & 0xF, layer=(c >> 21) & 0x3F,
                doff=(c >> 13) & 0x3F)


def load_case(vdir, name):
    with open(os.path.join(vdir, "gemm_%s_cmd.txt" % name)) as f:
        t = f.read().split()
    cmd64 = int(t[0], 16)
    layer, tag, n, k, beats, nstep, nused = (int(x, 16) for x in t[1:8])
    with open(os.path.join(vdir, "gemm_%s_drain.txt" % name)) as f:
        d = [int(x) for x in f.read().split()]
    ref_mode, ref_sat = d[4], d[5]
    assert ref_mode == 0, "%s: the board run checks INT16 flag-free outputs only (ref_mode %d)" % (name, ref_mode)
    cf = cmd_fields(cmd64)
    assert cf["dbuf"] == BUF_STG16 and cf["layer"] == layer and cf["tag"] == tag, name
    # the Stage-1 command: the case's word, flag-free, reading the window (as tb_me_top_s1)
    c = cmd64 & ~(0x3FF << 48)
    c = (c & ~(0xF << 44)) | (BUF_WIN << 44)
    return dict(cmd=c, layer=layer, tag=tag, n=n, k=k, beats=beats, nstep=nstep, nused=nused, ref_sat=ref_sat,
                iclass=cf["iclass"], doff=cf["doff"],
                stream=read_mem(os.path.join(vdir, "gemm_%s_stream.mem" % name), 512),
                win=read_mem(os.path.join(vdir, "gemm_%s_win.mem" % name), 512),
                absmax=read_mem(os.path.join(vdir, "gemm_%s_absmax.mem" % name), 16),
                shift=read_mem(os.path.join(vdir, "gemm_%s_shift.mem" % name), 8),
                ref=read_mem(os.path.join(vdir, "gemm_%s_ref.mem" % name), 32))


def window_stream(c):
    """The loader's beats: the constants beat, then each row's K/16 chunks, 4 a beat (tb_me_top_s1)."""
    kch = c["k"] // 16
    b = 0
    for r in range(c["nused"]):
        b |= (c["absmax"][r] & 0x7FFF) << (20 * r)
        b |= (c["shift"][r] & 0x1F) << (20 * r + 15)
    out = [b]
    m128 = (1 << 128) - 1
    for r in range(c["nused"]):
        for ch in range(0, kch, 4):
            b = 0
            for j in range(4):
                if ch + j < kch:
                    e = c["win"][(r // 4) * 96 + ch + j]
                    b |= ((e >> (128 * (r % 4))) & m128) << (128 * j)
            out.append(b)
    return out


def ref_beats(c):
    """The expected dump beats that hold outputs: row i, beat 2 DOFF + g holds channels 32g .. 32g + 31."""
    n, out = c["n"], []
    assert n % 32 == 0
    for i in range(c["nused"]):
        for g in range(n // 32):
            b = 0
            for j in range(32):
                v = c["ref"][i * n + 32 * g + j]
                lo = v & 0xFFFF
                assert v == (lo | (0xFFFF0000 if lo & 0x8000 else 0)), "reference %d does not fit INT16" % v
                b |= lo << (16 * j)
            out.append(b)
    return out


def script(c, classes):
    s = []
    wr = lambda a, d: s.append((OP_WR, a, d & 0xFFFFFFFF))
    wr(A_STEP, c["nstep"] | (1 << 8) | (30 << 16))         # the step's rows, 1 logit row, 30 layers
    wr(A_LOGIT_LO, c["nstep"] - 1)                          # logit row 0 = the last step row
    wr(A_LOGIT_HI, 0)
    for cid, e, w in classes:
        wr(A_CLASS, cid | ((e & 0xFF) << 8) | ((w & 0xFF) << 16))
    s.append((OP_GO, A_GO, 0x8))                            # CLR
    kch = c["k"] // 16
    wr(A_WIN, c["iclass"] | (c["nused"] << 8) | (kch << 16))
    wr(A_WIN_ADDR, WIN_DDR)
    s.append((OP_GO, A_GO, 0x2))                            # WIN
    wr(A_CMD_LO, c["cmd"])
    wr(A_CMD_HI, c["cmd"] >> 32)
    s.append((OP_GO, A_GO, 0x1))                            # GEMM
    s.append((OP_RD, A_CYCLES, 0))
    s.append((OP_RD, A_SAT, 1))
    s.append((OP_RD, A_BEATS, 2))
    wr(A_DUMP, 0)                                           # staging from word 0
    wr(A_DUMP_BEATS, c["nused"] * ROW_BEATS)
    wr(A_DUMP_ADDR, DUMP_DDR)
    s.append((OP_GO, A_GO, 0x4))                            # DUMP
    s.append((OP_END, 0, 0))
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vec", required=True, help="the vector files, one folder r<rows> per row count")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    classes = []
    with open(os.path.join(a.vec, "r5", "drain_classes.txt")) as f:
        for l in f:
            t = l.split()
            if len(t) == 4:
                classes.append((int(t[0]), int(t[2]), int(t[3])))

    wt, win, ref, scr, tab, notes = [], [], [], [], [], []
    wt_base = {}
    for sel in range(8):
        name, rows = LAYERS[sel >> 2], ROWS[sel & 3]
        c = load_case(os.path.join(a.vec, "r%d" % rows), name)
        assert c["nused"] == rows and c["nstep"] == rows, (name, rows)
        if name not in wt_base:                              # one weight ROM region per layer
            wt_base[name] = (len(wt), c["stream"])
            wt.extend(c["stream"])
        assert c["stream"] == wt_base[name][1], "%s: the weight stream differs at %d rows" % (name, rows)
        ws = window_stream(c)
        kch = c["k"] // 16
        assert len(ws) == 1 + c["nused"] * ((kch + 3) // 4)
        rb = ref_beats(c)
        f = dict(script_base=len(scr), win_base=len(win), win_beats=len(ws), wt_base=wt_base[name][0],
                 wt_beats=c["beats"], run_n=c["n"], run_k=c["k"], layer=c["layer"], tag=c["tag"],
                 ref_base=len(ref), ref_beats=len(rb), dump_beats=c["nused"] * ROW_BEATS, rel_lo=2 * c["doff"],
                 rel_n=c["n"] // 32, exp_sat=c["ref_sat"],
                 exp_arr_beats=(c["n"] // 32) * 4 * ((c["nused"] + 3) // 4) * kch)
        assert f["rel_lo"] + f["rel_n"] <= ROW_BEATS
        assert len(c["stream"]) == c["beats"]
        win.extend(ws)
        ref.extend(rb)
        scr.extend(script(c, classes))
        tab.extend([f[x] for x in CASE_FIELDS] + [0] * (CASE_LINES - len(CASE_FIELDS)))
        notes.append("sel %d: %-5s %2d rows  N %d K %d  weights %d beats  window %d beats  reference %d beats  "
                     "dump %d beats  array beats %d  saturations %d" %
                     (sel, name, rows, c["n"], c["k"], c["beats"], len(ws), len(rb), f["dump_beats"],
                      f["exp_arr_beats"], c["ref_sat"]))

    def wmem(fn, vals, bits):
        with open(os.path.join(a.out, fn), "w", newline="\n") as f:
            for v in vals:
                f.write(hexw(v, bits) + "\n")

    wmem("bd_wt.mem", wt, 512)
    for b in range(0, len(wt), 1024):
        wmem("bd_wt_b%d.mem" % (b // 1024), wt[b:b + 1024] + [0] * max(0, b + 1024 - len(wt)), 512)
    wmem("bd_win.mem", win, 512)
    wmem("bd_ref.mem", ref, 512)
    wmem("bd_script.mem", [(op << 62) | ((ad & 0xFFF) << 32) | d for op, ad, d in scr], 64)
    wmem("bd_case.mem", tab, 32)
    with open(os.path.join(a.out, "bd_cases.txt"), "w", newline="\n") as f:
        f.write("\n".join(notes) + "\n")
        f.write("ROM depths: weights %d, window %d, reference %d, script %d (512/512/512/64 bits)\n" %
                (len(wt), len(win), len(ref), len(scr)))
    print("\n".join(notes))
    print("ROM depths: weights %d, window %d, reference %d, script %d" % (len(wt), len(win), len(ref), len(scr)))


if __name__ == "__main__":
    main()
