#!/usr/bin/env python3
"""
File: check_gemm_vectors.py
Block: Matrix engine - independent check of the vector files (vectors/r5, vectors/r16)

Reads only the vector files: decodes each stream by the interface's layout
(per 32-channel group: one scale beat, then tiles ct = 0..3, chunks kc fastest, slot byte
16c + d = W[8ct + c][16kc + d]), the window by row r chunk c at line r*96 + c, and recomputes
acc = X8 . W^T for the used rows. Also checks that scales.mem is each group's first beat and that
the tile order the array sees (group, tile, row group, chunk) reproduces the same sums.
With the drain's files present, recomputes each output one at a time from the drain's formulas
(column scale, absmax, row shift, class exponents, round half even, saturation, ACC): the flag-free
references, the reference model's ACC outputs, the edge cases and their saturation counts.

  python golden/check_gemm_vectors.py <dir>
"""

import os
import sys
import numpy as np


def rd_hex(path, bits):
    vals = []
    for line in open(path):
        line = line.strip()
        if line:
            v = int(line, 16)
            if v >> (bits - 1):
                v -= 1 << bits
            vals.append(v)
    return vals


def rd_bytes(path):
    return [bytes.fromhex(l.strip())[::-1] for l in open(path) if l.strip()]   # byte 0 first


# ---- the drain, scalar, from the drain's formulas ----
def fp16_mant_exp(h):
    ex, fr = (h >> 10) & 0x1F, h & 0x3FF
    return (fr, -24) if ex == 0 else (fr | 0x400, ex - 25)


def norm17(v):
    if v == 0:
        return 0, 0
    n = max(0, v.bit_length() - 17)
    m = (v + (1 << (n - 1))) >> n if n else v
    if m >> 17:
        m, n = m >> 1, n + 1
    return m, n


def drain_one(acc, scale, absmax, e_in, sh, e_out, width, dest=None):
    """One output, and whether it saturated: y = sat(rhe(acc * c17, shift)) (+ dest, saturated)."""
    m_col, e_col = fp16_mant_exp(scale)
    kc17, nc = norm17(m_col * 132104 * 1024)            # 132104 = round(2^24 / 127); 1/s_x = 1.0 = 1024 x 2^-10
    c17, n2 = norm17(kc17 * absmax)
    s = 15 + e_out - (e_col - 10) - e_in - sh + 24 - nc - n2
    p = acc * c17
    if s > 0:
        q, rem, half = p >> s, p - ((p >> s) << s), 1 << (s - 1)
        v = q + (rem > half or (rem == half and q & 1))
    else:
        v = p << -s
    lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1
    hit = not lo <= v <= hi
    v = max(lo, min(hi, v))
    if dest is not None:
        hit |= not lo <= v + dest <= hi
        v = max(lo, min(hi, v + dest))
    return v, hit


def scales_of(path):
    return [int.from_bytes(b[2 * j:2 * j + 2], "little") for b in rd_bytes(path) for j in range(32)]


def check_drain(acc, scol, absmax, shift, e_in, e_out, width, ref, dest=None, nsat=None):
    nr, n = acc.shape
    bad, sat = 0, 0
    for r in range(nr):
        for ch in range(n):
            v, hit = drain_one(int(acc[r, ch]), scol[ch], int(absmax[r]), e_in, int(shift[r]), e_out, width,
                               None if dest is None else int(dest[r, ch]))
            bad += v != ref[r * n + ch]
            sat += hit
    return bad == 0 and (nsat is None or sat == nsat), bad, sat


def main_drain(d):
    """The drain's files: the flag-free references, the ACC outputs, the edge cases and the saturation counts."""
    fails = 0
    cls = {}
    for line in open(os.path.join(d, "drain_classes.txt")):
        cid, name, e, w = line.split()
        cls[int(cid)] = (name, int(e), int(w))
    for line in open(os.path.join(d, "manifest.txt")):
        name, n, k, nused = line.split()[:4]
        n, nused = int(n), int(nused)
        f = "gemm_%s_" % name
        ic, oc, e_in, e_out, ref_mode, ref_sat, out_check, out_sat = map(int, open(os.path.join(d, f + "drain.txt")).read().split())
        assert (e_in, e_out) == (cls[ic][1], cls[oc][1])
        acc = np.asarray(rd_hex(os.path.join(d, f + "acc.mem"), 34), np.int64).reshape(nused, n)
        scol = scales_of(os.path.join(d, f + "scales.mem"))
        absmax = rd_hex(os.path.join(d, f + "absmax.mem"), 17)
        shift = rd_hex(os.path.join(d, f + "shift.mem"), 9)
        msg = []
        ok = True
        if ref_mode >= 0:
            assert ref_mode == (2 if cls[oc][2] == 32 else 0)
            ref = rd_hex(os.path.join(d, f + "ref.mem"), 32)
            r_ok, bad, sat = check_drain(acc, scol, absmax, shift, e_in, e_out, 32 if ref_mode == 2 else 16, ref,
                                         nsat=ref_sat)
            ok &= r_ok
            msg.append("flag-free %s (%d wrong, %d saturated)" % ("ok" if r_ok else "BAD", bad, sat))
        else:
            msg.append("flag-free none (K/V)")
        if out_check:
            out = rd_hex(os.path.join(d, f + "out.mem"), 32)
            dp = os.path.join(d, f + "dest.mem")
            dest = np.asarray(rd_hex(dp, 32), np.int64).reshape(nused, n) if os.path.exists(dp) else None
            o_ok, bad, sat = check_drain(acc, scol, absmax, shift, e_in, e_out, cls[oc][2], out, dest, out_sat)
            ok &= o_ok
            msg.append("reference out%s %s (%d wrong, %d saturated)" % (" with ACC" if dest is not None else "",
                                                                         "ok" if o_ok else "BAD", bad, sat))
        fails += not ok
        print("%-4s drain %-12s %s" % ("PASS" if ok else "FAIL", name, "; ".join(msg)))
    for line in open(os.path.join(d, "drain_manifest.txt")):
        name, nr, n, ic, oc, mode, acc_on, fault, nsat = line.split()
        nr, n, ic, oc, mode, acc_on, fault, nsat = int(nr), int(n), int(ic), int(oc), int(mode), int(acc_on), int(fault, 16), int(nsat)
        f = "drain_%s_" % name
        acc = np.asarray(rd_hex(os.path.join(d, f + "acc.mem"), 34), np.int64).reshape(nr, n)
        scol = scales_of(os.path.join(d, f + "scales.mem"))
        absmax = rd_hex(os.path.join(d, f + "absmax.mem"), 17)
        shift = rd_hex(os.path.join(d, f + "shift.mem"), 9)
        bad_sc = [j for j, h in enumerate(scol) if (h >> 10) & 0x1F == 0x1F]
        if fault:
            ok = fault == 0x42 and len(bad_sc) > 0
            print("%-4s drain %-15s SCALE fault expected, malformed scale at channel %s" % (
                "PASS" if ok else "FAIL", name, bad_sc))
        else:
            dest = np.asarray(rd_hex(os.path.join(d, f + "dest.mem"), 32), np.int64).reshape(nr, n) if acc_on else None
            ref = rd_hex(os.path.join(d, f + "ref.mem"), 32)
            ok, bad, sat = check_drain(acc, scol, absmax, shift, cls[ic][1], cls[oc][1], 32 if mode == 2 else 16,
                                       ref, dest, nsat)
            ok &= not bad_sc and mode == (2 if cls[oc][2] == 32 else 0)
            print("%-4s drain %-15s %s->%s %s%s: %d wrong, %d saturated" % (
                "PASS" if ok else "FAIL", name, cls[ic][0], cls[oc][0], "INT32" if mode == 2 else "INT16",
                " ACC" if acc_on else "", bad, sat))
        fails += not ok
    return fails


def main(d):
    fails = 0
    for line in open(os.path.join(d, "manifest.txt")):
        name, n, k, nused, beats, groups, nout = line.split()
        n, k, nused, beats, groups = int(n), int(k), int(nused), int(beats), int(groups)
        f = open(os.path.join(d, "gemm_%s_cmd.txt" % name)).read().split()
        nstep, mask = int(f[6], 16), int(f[8], 16)
        rows = [r for r in range(nstep) if mask >> r & 1]
        assert len(rows) == nused
        st = rd_bytes(os.path.join(d, "gemm_%s_stream.mem" % name))
        assert len(st) == beats and all(len(b) == 64 for b in st)
        kch = k // 16
        W = np.zeros((n, k), np.int64)
        scales = []
        p = 0
        for g in range(groups):
            scales.append(st[p]); p += 1
            for ct in range(4):
                for kc in range(kch):
                    slot = st[p] + st[p + 1]; p += 2
                    for c in range(8):
                        for dd in range(16):
                            W[32 * g + 8 * ct + c, 16 * kc + dd] = slot[16 * c + dd] - 256 * (slot[16 * c + dd] >> 7)
        assert p == beats
        sc = rd_bytes(os.path.join(d, "gemm_%s_scales.mem" % name))
        ok_sc = sc == scales
        win = rd_bytes(os.path.join(d, "gemm_%s_win.mem" % name))      # packed: used row i = 4rg + lane
        assert len(win) == 576
        X = np.zeros((24, 1536), np.int64)
        for rg in range(6):
            for c in range(96):
                for r in range(4):
                    X[4 * rg + r, 16 * c:16 * c + 16] = [b - 256 * (b >> 7) for b in win[rg * 96 + c][16 * r:16 * r + 16]]
        acc_ref = np.asarray(rd_hex(os.path.join(d, "gemm_%s_acc.mem" % name), 34), np.int64).reshape(nused, n)
        rows = list(range(nused))                                          # window rows
        acc = X[rows, :k] @ W.T
        ok_acc = np.array_equal(acc, acc_ref)
        # the array's view: per tile (g, ct), per row group of 4 used rows, sum over chunks
        ok_tile = True
        for g in range(groups):
            for ct in range(4):
                ch = slice(32 * g + 8 * ct, 32 * g + 8 * ct + 8)
                for rg in range(0, nused, 4):
                    rr = rows[rg:rg + 4]
                    t = sum(X[rr, 16 * kc:16 * kc + 16] @ W[ch, 16 * kc:16 * kc + 16].T for kc in range(kch))
                    ok_tile &= np.array_equal(t, acc_ref[rg:rg + 4, ch])
        bad = not (ok_sc and ok_acc and ok_tile)
        fails += bad
        print("%-4s %-12s N %5d K %5d rows %d beats %6d  scales %s  acc %s  tiles %s  max|acc| %d" % (
            "FAIL" if bad else "PASS", name, n, k, nused, beats, ok_sc, ok_acc, ok_tile, int(np.abs(acc_ref).max())))
    if os.path.exists(os.path.join(d, "drain_manifest.txt")):
        fails += main_drain(d)
    print("ALL PASS" if fails == 0 else "FAILURES: %d" % fails)
    return fails


if __name__ == "__main__":
    sys.exit(1 if main(sys.argv[1]) else 0)
