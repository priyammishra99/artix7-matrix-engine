#!/usr/bin/env python3
"""
File: gen_me_vectors.py
Block: Matrix engine - golden vector generator for tb_me_array (contract v2)

Reference model of the R4 x C8 x D16 array in its four modes:

  DSP d of cell (r, j):   (A[j][d] + Dp[d]) * X[r][d]      cell sum over d = 0..15

  GEMM                    A = INT8 weight  W[8g+j][16c+d],  Dp = 0,  X = INT8 activation
                          tile (g) of K/16 beats -> acc(r, j) = sum_k X[r][k] W[8g+j][k]
                          handshake mode (credits, drain may stall)
  score                   keys in slots of 8 keys (columns) x 16 dims (cascade)
                          sealed page:  A = code x r_dim,  Dp = 15 mn_dim   (K15 = A + Dp)
                          INT8 page:    A = 15 k8,         Dp = 0
                          X = q16; a tile = 4 beats (the 4 chunks of 64 dims) -> 34-bit raw scores
  P.V                     values in slots of 8 dims (columns) x 16 keys (cascade)
                          sealed page:  A = code x r_tok,  Dp = 15 mn_tok   (V15 = A + Dp)
                          X = p_lo (<= 4,095) or p_hi (<= 2,048); BYPASS beats: each beat's
                          32 cell sums leave on the bypass port with the beat's tag
  page scan               A = 8h + 3 (h: the 2-bit high key code), Dp = 0,
                          X = folded query q8 x r, |.| <= 32,258; tiles of 4 beats
  Attention and scan run in push mode (contract rule P10).

Files written for each case NAME (in --out):
  me_NAME_beats.mem   one line per beat, 724 hex digits:
                      {ctrl[15:0], x[1023:0], dp[191:0], w[1663:0]}
                      ctrl: bit 0 first, bit 1 last, bit 2 bypass, bits 15:8 tag
                      w element (j*16+d) 13 bits, dp element d 12 bits,
                      x element (r*16+d) 16 bits, all two's complement
  me_NAME_tiles.mem   one line per tile, 272 hex: 32 x 34-bit, acc (r, j) at [(r*8+j)*34 +: 34]
  me_NAME_byp.mem     one line per bypass beat, 250 hex: {tag[7:0], 32 x 31-bit sums}
and manifest.txt:     name  beats  tiles  bypass_beats  mode(0 handshake, 1 push)
"""

import argparse
import os
import numpy as np

R, C, D = 4, 8, 16
A_W, DP_W, X_W = 13, 12, 16
SUM_W, ACC_W, TAG_W = 31, 34, 8
W_BITS, DP_BITS, X_BITS = C * D * A_W, D * DP_W, R * D * X_W   # 1664, 192, 1024
NACC = R * C


def pack(vals, width):
    """vals[0] in the least significant field; two's complement in `width` bits."""
    word, mask = 0, (1 << width) - 1
    for i, v in enumerate(vals):
        word |= (int(v) & mask) << (i * width)
    return word


def fits(v, width):
    return -(1 << (width - 1)) <= v < (1 << (width - 1))


class Case:
    def __init__(self, name, mode):
        self.name, self.mode = name, mode
        self.beats, self.tiles, self.byp = [], [], []

    def beat(self, A, Dp, X, first=0, last=0, bypass=0, tag=0):
        """A: (C, D), Dp: (D,), X: (R, D) integer arrays. Returns the cell sums (R, C)."""
        A, Dp, X = np.asarray(A, dtype=np.int64), np.asarray(Dp, dtype=np.int64), np.asarray(X, dtype=np.int64)
        assert all(fits(int(v), A_W) for v in A.flat), "A out of range"
        assert all(fits(int(v), DP_W) for v in Dp.flat), "Dp out of range"
        assert all(fits(int(v), X_W) for v in X.flat), "X out of range"
        S = (A + Dp[None, :])                       # (C, D): what the pre-adder forms
        sums = X @ S.T                              # (R, C): the cell sums
        assert all(fits(int(v), SUM_W) for v in sums.flat), "cell sum exceeds 31 bits"
        ctrl = (first & 1) | ((last & 1) << 1) | ((bypass & 1) << 2) | ((tag & 0xFF) << 8)
        word = pack(A.reshape(-1), A_W)
        word |= pack(Dp, DP_W) << W_BITS
        word |= pack(X.reshape(-1), X_W) << (W_BITS + DP_BITS)
        word |= ctrl << (W_BITS + DP_BITS + X_BITS)
        self.beats.append(word)
        if bypass:
            self.byp.append(((tag & 0xFF) << (NACC * SUM_W)) | pack(sums.reshape(-1), SUM_W))
        return sums

    def tile(self, beats):
        """beats: list of (A, Dp, X); one accumulated tile, its result appended."""
        acc = np.zeros((R, C), dtype=np.int64)
        n = len(beats)
        for i, (A, Dp, X) in enumerate(beats):
            acc += self.beat(A, Dp, X, first=int(i == 0), last=int(i == n - 1))
        assert all(fits(int(v), ACC_W) for v in acc.flat), "accumulator exceeds 34 bits"
        self.tiles.append(pack(acc.reshape(-1), ACC_W))

    def write(self, outdir):
        p = lambda s: os.path.join(outdir, f"me_{self.name}_{s}.mem")
        with open(p("beats"), "w") as f:
            f.writelines(f"{b:0724x}\n" for b in self.beats)
        with open(p("tiles"), "w") as f:
            f.writelines(f"{t:0272x}\n" for t in self.tiles)
            if not self.tiles:
                f.write("0" * 272 + "\n")           # keep $readmemh happy
        with open(p("byp"), "w") as f:
            f.writelines(f"{y:0250x}\n" for y in self.byp)
            if not self.byp:
                f.write("0" * 250 + "\n")
        return f"{self.name} {len(self.beats)} {len(self.tiles)} {len(self.byp)} {self.mode}"


# ---------------------------------------------------------------------------- GEMM
def gemm_case(name, K, N, kind, seed):
    rng = np.random.default_rng(seed)
    if kind == "random":
        W, X = rng.integers(-127, 128, (N, K)), rng.integers(-127, 128, (R, K))
    elif kind == "max":
        W, X = np.full((N, K), 127), np.full((R, K), 127)
    elif kind == "min":
        W, X = np.full((N, K), -127), np.full((R, K), 127)
    elif kind == "neg128":                          # not produced by the quantisers; still exact
        W, X = np.full((N, K), -128), np.full((R, K), -128)
    elif kind == "alt":
        k = np.arange(K)
        W = np.stack([127 * np.where((k + n) % 2 == 0, 1, -1) for n in range(N)])
        X = np.stack([127 * np.where((k // 3 + r) % 2 == 0, 1, -1) for r in range(R)])
    cs = Case(name, 0)
    zero_dp = np.zeros(D, dtype=np.int64)
    for g in range(N // C):
        cs.tile([(W[C * g:C * g + C, D * c:D * c + D], zero_dp, X[:, D * c:D * c + D])
                 for c in range(K // D)])
    return cs


# ---------------------------------------------------------------------------- attention
def sealed_keys(rng, ntok=16, ndim=64):
    """A page of keys in 15x units: per-channel ranges              . Returns A (tok, dim), Dp (dim)."""
    mn = rng.integers(-127, 128, ndim)
    mx = np.minimum(127, mn + rng.integers(0, 255, ndim))
    r = mx - mn
    code = rng.integers(0, 16, (ntok, ndim))
    return code * r[None, :], 15 * mn


def int8_keys(rng, ntok=16, ndim=64):
    return 15 * rng.integers(-127, 128, (ntok, ndim)), np.zeros(ndim, dtype=np.int64)


def score_block(cs, A, Dp, Q):
    """One 16-key block: A (16 keys, 64 dims), Dp (64 dims), Q (R, 64): 2 tiles of 4 beats."""
    for half in range(2):                           # keys 0-7, keys 8-15 (2 column passes)
        keys = slice(8 * half, 8 * half + 8)
        cs.tile([(A[keys, 16 * c:16 * c + 16], Dp[16 * c:16 * c + 16], Q[:, 16 * c:16 * c + 16])
                 for c in range(4)])


def score_case(name, nblocks, seed, extreme=False):
    rng = np.random.default_rng(seed)
    cs = Case(name, 1)
    for b in range(nblocks):
        if extreme:
            # K15 at its limits, A at its maximum 3,810 (mn = -127, r = 254, code 15): K15 = +1,905
            A = np.full((16, 64), 3810); Dp = np.full(64, -1905)
            if b % 2:
                A = np.zeros((16, 64), dtype=np.int64); Dp = np.full(64, -1905)   # K15 = -1,905
            Q = np.full((R, 64), 32767) if b % 4 < 2 else np.full((R, 64), -32767)
        else:
            A, Dp = sealed_keys(rng) if rng.random() < 0.75 else int8_keys(rng)
            Q = rng.integers(-32767, 32768, (R, 64))
        score_block(cs, A, Dp, Q)
    return cs


def sealed_values(rng, ntok=16, ndim=64):
    """Values in 15x units: per-token ranges              . Returns A (tok, dim), Dp (tok)."""
    mn = rng.integers(-127, 128, ntok)
    mx = np.minimum(127, mn + rng.integers(0, 255, ntok))
    code = rng.integers(0, 16, (ntok, ndim))
    return code * (mx - mn)[:, None], 15 * mn


def pv_pass(cs, A, Dp, P, tag0):
    """One block's P.V pass: 8 column passes (8 dims each) x 2 halves of p = 16 bypass beats.
    A (16 tok, 64 dims), Dp (16 tok), P (R, 16 tok) full 24-bit probabilities."""
    halves = [(P & 0xFFF), (P >> 12)]               # p_lo <= 4,095, p_hi <= 2,048
    t = tag0
    for col in range(8):
        dims = slice(8 * col, 8 * col + 8)
        for h in range(2):
            cs.beat(A[:, dims].T, Dp, halves[h], bypass=1, tag=t)   # slot: 8 dims x 16 keys
            t = (t + 1) & 0xFF
    return t


def pv_case(name, nblocks, seed, extreme=False):
    rng = np.random.default_rng(seed)
    cs = Case(name, 1)
    t = 0
    for b in range(nblocks):
        if extreme:
            A = np.full((16, 64), 3810) if b % 2 == 0 else np.zeros((16, 64), dtype=np.int64)
            Dp = np.full(16, -1905)                     # V15 = +1,905 / -1,905
            P = np.full((R, 16), (2048 << 12) | 4095)   # p_hi = 2,048 (p = 1.0 region), p_lo = 4,095
        else:
            A, Dp = sealed_values(rng)
            P = rng.integers(0, 1 << 23, (R, 16)) + 1   # UQ1.23, up to 2^23 (p = 1.0)
            P = np.minimum(P, 1 << 23)
        t = pv_pass(cs, A, Dp, P, t)
    return cs


def mixed_case(name, nblocks, seed):
    """Attention's schedule             : score passes run two blocks ahead of P.V passes."""
    rng = np.random.default_rng(seed)
    cs = Case(name, 1)
    blocks = []
    for b in range(nblocks):
        kA, kD = sealed_keys(rng) if b % 3 else int8_keys(rng)
        vA, vD = sealed_values(rng)
        Q = rng.integers(-32767, 32768, (R, 64))
        P = np.minimum(rng.integers(0, 1 << 23, (R, 16)) + 1, 1 << 23)
        blocks.append((kA, kD, Q, vA, vD, P))
    t = 0
    for b in range(nblocks + 2):
        if b < nblocks:
            kA, kD, Q = blocks[b][:3]
            score_block(cs, kA, kD, Q)
        if b >= 2:
            vA, vD, P = blocks[b - 2][3:]
            t = pv_pass(cs, vA, vD, P, t)
    return cs


def scan_case(name, npages, seed, extreme=False):
    """Page scan              : A = 8h + 3, Dp = 0, X = folded query q8 x r."""
    rng = np.random.default_rng(seed)
    cs = Case(name, 1)
    for pg in range(npages):
        if extreme:
            h = np.full((16, 64), 3); Q = np.full((R, 64), 32258 if pg % 2 == 0 else -32258)
        else:
            h = rng.integers(0, 4, (16, 64))
            q8 = rng.integers(-127, 128, (R, 64)); rr = rng.integers(0, 255, 64)
            Q = q8 * rr[None, :]
        score_block(cs, 8 * h + 3, np.zeros(64, dtype=np.int64), Q)
    return cs


def push_one_case(name, ntiles, seed):
    """One-beat tiles back to back in push mode: a bank is reused every second cycle."""
    rng = np.random.default_rng(seed)
    cs = Case(name, 1)
    for _ in range(ntiles):
        cs.tile([(rng.integers(-127, 128, (C, D)), np.zeros(D, dtype=np.int64),
                  rng.integers(-127, 128, (R, D)))])
    return cs


def all_cases():
    return [
        # GEMM, handshake mode (as v1, same shapes)
        gemm_case("wq",     576,  576, "random", 1),   # layer-0 W_Q: 72 tiles x 36 chunks
        gemm_case("max",    1536, 8,   "max",    2),   # K = 1,536 (W_down), one tile
        gemm_case("min",    1536, 8,   "min",    3),
        gemm_case("neg128", 1536, 8,   "neg128", 4),
        gemm_case("alt",    576,  64,  "alt",    5),
        gemm_case("one",    16,   64,  "random", 6),   # one-beat tiles
        gemm_case("two",    32,   64,  "random", 7),
        gemm_case("w_o",    576,  192, "random", 8),   # W_K / W_V shape
        # attention and scan, push mode (contract v2)
        score_case("score",     24, 21),               # 48 tiles of 4 beats, sealed + INT8 pages
        score_case("score_max",  8, 22, extreme=True), # raw scores at +-64 x 32,767 x 1,905
        pv_case("pv",           16, 23),               # 256 bypass beats
        pv_case("pv_max",        4, 24, extreme=True), # cell sums at +-16 x 4,095 x 1,905 and p_hi = 2,048
        mixed_case("mixed",     12, 25),               # score passes two blocks ahead of P.V
        scan_case("scan",       16, 26),
        scan_case("scan_max",    4, 27, extreme=True),
        push_one_case("push_one", 64, 28),             # 1-beat tiles every cycle in push mode
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="output directory")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    lines = []
    for cs in all_cases():
        lines.append(cs.write(args.out))
        print(f"  {cs.name:10s} mode {cs.mode}: {len(cs.beats):5d} beats, {len(cs.tiles):4d} tiles, "
              f"{len(cs.byp):4d} bypass beats")
    with open(os.path.join(args.out, "manifest.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
